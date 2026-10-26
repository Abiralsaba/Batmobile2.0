"""Diagnostic Pi interface only: no serial, sockets, Pi or motor hardware."""
import json
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

try:
    import rclpy
    from std_msgs.msg import String
    from rover.pi_bridge_demo import GuidanceMonitor, PiBridgeDemo
except ImportError:
    rclpy = None

from rover.mission_control_node import FollowController
from rover.vision import FRAME_ID


def guidance(**changes):
    data = dict(state='FOLLOW', person_id=7, distance_m=3., desired_m=2.,
                distance_error_m=1., tracking_valid=True, linear_mps=.2,
                angular_rps=.1, reason='FOLLOWING', source_frame=FRAME_ID,
                calibration_status='CALIBRATION_REQUIRED', motors_enabled=False,
                reset_required=False)
    data.update(changes)
    return data


@unittest.skipIf(rclpy is None, 'Source ROS Humble to run demo tests')
class TestGuidanceMonitor(unittest.TestCase):
    def setUp(self):
        self.monitor = GuidanceMonitor()
        self.wall, self.mono = 1000., 10.

    def receive(self, data):
        self.monitor.receive(json.dumps(data), wall=self.wall, mono=self.mono)

    def status(self):
        result = self.monitor.snapshot(wall=self.wall, mono=self.mono)
        json.dumps(result, allow_nan=False)
        for key in ('pi_connected', 'motors_enabled', 'command_executed', 'action_allowed'):
            self.assertIs(result[key], False)
        self.assertEqual(result['mode'], 'DRY_RUN')
        return result

    def test_no_input_has_zero_diagnostics(self):
        result = self.status()
        self.assertFalse(result['guidance_received'])
        self.assertEqual(result['requested_linear_mps'], 0.)
        self.assertEqual(result['reason'], 'NO_GUIDANCE')

    def test_current_guidance_reports_intentions_but_source_age_unknown(self):
        self.receive(guidance())
        result = self.status()
        self.assertTrue(result['guidance_received'])
        self.assertTrue(result['guidance_valid'])
        self.assertEqual(result['requested_linear_mps'], .2)
        self.assertEqual(result['requested_angular_rps'], .1)
        self.assertFalse(result['source_freshness_verified'])
        self.assertIn('CALIBRATION_REQUIRED', result['blocked_reasons'])
        self.assertIn('SOURCE_TIME_UNAVAILABLE', result['blocked_reasons'])

    def test_unchanged_existing_controller_schema(self):
        controller = FollowController()
        self.receive(controller.guidance(wall=self.wall, mono=self.mono))
        self.assertTrue(self.status()['guidance_valid'])
        self.assertEqual(self.status()['requested_linear_mps'], 0.)

    def test_monotonic_receipt_timeout_zeros_requested_values(self):
        self.receive(guidance())
        self.mono += .51  # Wall/ROS time frozen.
        result = self.status()
        self.assertFalse(result['guidance_valid'])
        self.assertEqual(result['reason'], 'STALE_GUIDANCE')
        self.assertEqual((result['requested_linear_mps'], result['requested_angular_rps']), (0., 0.))

    def test_stamped_data_checked_for_age_future_and_replay(self):
        for stamp in (self.wall-.6, self.wall+.2, True, float('nan')):
            self.receive(guidance(timestamp=stamp))
            self.assertFalse(self.status()['guidance_valid'])
        self.receive(guidance(timestamp=self.wall))
        self.assertTrue(self.status()['source_freshness_verified'])
        self.receive(guidance(timestamp=self.wall))
        self.assertFalse(self.status()['guidance_valid'])
        self.assertEqual(self.status()['reason'], 'STALE_OR_REPLAYED_GUIDANCE')

    def test_source_age_consumes_monotonic_deadline(self):
        self.receive(guidance(timestamp=self.wall-.4))
        self.mono += .11
        self.wall -= .01
        self.assertFalse(self.status()['guidance_valid'])

    def test_every_required_field_and_bad_types_rejected(self):
        for key in guidance():
            data = guidance()
            del data[key]
            self.receive(data)
            self.assertFalse(self.status()['guidance_valid'], key)
        for changes in ({'motors_enabled': True}, {'person_id': True}, {'person_id': -1},
                        {'linear_mps': .21}, {'linear_mps': -.1}, {'linear_mps': True},
                        {'angular_rps': .31}, {'angular_rps': float('inf')},
                        {'state': 'OTHER'}, {'tracking_valid': 1}, {'reset_required': None},
                        {'calibration_status': None}, {'source_frame': 'base_link'},
                        {'distance_error_m': .4}, {'distance_m': None}, {'desired_m': 0}):
            self.receive(guidance(**changes))
            self.assertFalse(self.status()['guidance_valid'], changes)

    def test_nonzero_guidance_requires_follow_and_valid_target(self):
        for changes in ({'state': 'SAFE_STOP'}, {'state': 'STAY_LOCKED'}, {'person_id': None},
                        {'tracking_valid': False}, {'reset_required': True}):
            self.receive(guidance(**changes))
            self.assertFalse(self.status()['guidance_valid'])

    def test_malformed_json_rejects_previous_valid_guidance(self):
        for text in ('null', '[]', '{', '{"a":NaN}', '{"a":Infinity}',
                     '{"a":1e999}', '{"a":1,"a":2}'):
            self.receive(guidance())
            self.monitor.receive(text, wall=self.wall, mono=self.mono)
            self.assertFalse(self.status()['guidance_valid'])
            self.assertEqual(self.status()['requested_linear_mps'], 0.)

    def test_even_calibrated_fresh_guidance_never_allows_actuation(self):
        self.receive(guidance(timestamp=self.wall, calibration_status='CALIBRATED'))
        self.assertTrue(self.status()['guidance_valid'])
        self.assertIn('NO_ACTUATOR_INTERFACE', self.status()['blocked_reasons'])


@unittest.skipIf(rclpy is None, 'Source ROS Humble to run demo tests')
class TestPiBridgeDemoNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.wall, self.mono = 1000., 10.
        self.time_patch = patch('rover.pi_bridge_demo.time',
                                SimpleNamespace(time=lambda: self.wall, monotonic=lambda: self.mono))
        self.time_patch.start()
        self.addCleanup(self.time_patch.stop)
        self.node = PiBridgeDemo()
        self.addCleanup(self.node.destroy_node)

    def test_only_diagnostic_publisher_and_guidance_subscription(self):
        topics = {p.topic_name for p in self.node.publishers}
        self.assertEqual(topics - {'/rosout', '/parameter_events'}, {'/rover/demo/pi_bridge/status'})
        self.assertIn('/rover/follow/guidance', {s.topic_name for s in self.node.subscriptions})

    def test_callback_and_timer_publish_strict_status_then_stale(self):
        self.node.pub = Mock()
        self.node.on_guidance(String(data=json.dumps(guidance())))
        result = json.loads(self.node.pub.publish.call_args.args[0].data)
        self.assertTrue(result['guidance_valid'])
        self.assertFalse(result['command_executed'])
        self.mono += .6
        self.node.publish_status()
        result = json.loads(self.node.pub.publish.call_args.args[0].data)
        self.assertEqual(result['reason'], 'STALE_GUIDANCE')
        self.assertEqual(result['requested_linear_mps'], 0.)
