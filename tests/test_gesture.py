"""FOLLOW baseline and node callbacks; no camera hardware or actuators needed.

Source ROS Humble to run this suite. Existing portable tests still run without ROS.
"""
import json
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

try:
    import rclpy
    from std_msgs.msg import String
    from rover.gesture import FollowMotion, GestureNode
except ImportError:
    rclpy = None


def person(pid, dy=.22, side='RIGHT'):
    points = [[0., 0., 2.5] for _ in range(18)]
    points[4][1] = .22
    points[7][1] = .22
    points[4 if side == 'RIGHT' else 7][1] = dy
    return {'id': pid, 'tracking_state': 'OK', 'x_m': 0., 'y_m': 0., 'z_m': 2.5,
            'distance_m': 2.5, 'bearing_deg': 0., 'keypoints_m': points}


@unittest.skipIf(rclpy is None, 'Source /opt/ros/humble/setup.bash')
class TestFollowMotion(unittest.TestCase):
    def test_requested_sample(self):
        d = FollowMotion()
        values = [.22]*8 + [-.14]*6 + [.22]*6
        self.assertEqual(sum(d.update(y, i*.15) for i,y in enumerate(values)), 1)

    def test_static_arm_is_not_follow_event(self):
        for value in (.22, -.14, 0.):
            d = FollowMotion()
            self.assertFalse(any(d.update(value, i*.1) for i in range(100)))

    def test_requires_down_up_down_order(self):
        d = FollowMotion()
        self.assertFalse(any(d.update(y, i*.1) for i,y in enumerate([-.14]*8+[.22]*8)))

    def test_timeout_and_discontinuous_samples_reset(self):
        for gap in (.8, 4.):
            d = FollowMotion()
            for i,y in enumerate([.22]*4+[-.14]*4):
                d.update(y, i*.1)
            self.assertFalse(any(d.update(.22, .7+gap+i*.1) for i in range(4)))

    def test_replayed_and_nonfinite_samples_reset(self):
        d = FollowMotion()
        self.assertFalse(d.update(.22, 1.))
        self.assertFalse(d.update(-.14, 1.))
        self.assertFalse(d.update(float('nan'), 2.))
        self.assertEqual(d.phase, 0)


@unittest.skipIf(rclpy is None, 'Source /opt/ros/humble/setup.bash')
class TestGestureNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.wall = 1000.
        self.mono = 10.
        self.clock = SimpleNamespace(time=lambda: self.wall, monotonic=lambda: self.mono)
        self.clock_patch = patch('rover.gesture.time', self.clock)
        self.clock_patch.start()
        self.node = GestureNode()
        self.node.select_pub = Mock()
        self.node.event_pub = Mock()
        self.control_state = 'WAIT_COMMAND'
        self.auto_guidance = True
        self.guidance()

    def tearDown(self):
        self.node.destroy_node()
        self.clock_patch.stop()

    def tick(self, dt=.1):
        self.wall += dt
        self.mono += dt

    def bodies(self, people, stamp=None):
        if self.auto_guidance:
            self.guidance()
        self.node.on_bodies(String(data=json.dumps({
            'timestamp': self.wall if stamp is None else stamp,
            'detected': bool(people), 'people': people})))

    def guidance(self, **updates):
        data = dict(state=self.control_state, published_timestamp=self.wall,
                    follow_gesture_allowed=self.control_state == 'WAIT_COMMAND')
        data.update(updates)
        self.node.on_guidance(String(data=json.dumps(data)))

    def gesture(self, ids=(5,), side='RIGHT'):
        for dy in [.22]*8 + [-.14]*6 + [.22]*6:
            self.bodies([person(3)] + [person(pid, dy, side) for pid in ids])
            self.tick()

    def target(self, **updates):
        data = {'valid': True, 'locked': True, 'tracking_state': 'OK',
                'person_id': 5, 'age_sec': .05, 'timestamp': self.wall-.05}
        data.update(updates)
        self.node.on_target(String(data=json.dumps(data)))

    def test_any_person_select_then_confirm_once(self):
        self.gesture()
        self.node.select_pub.publish.assert_called_once()
        command = json.loads(self.node.select_pub.publish.call_args.args[0].data)
        self.assertEqual(command, {'action': 'select', 'person_id': 5})
        self.node.event_pub.publish.assert_not_called()
        self.target(person_id=3)
        self.node.event_pub.publish.assert_not_called()
        self.target()
        self.target()
        self.node.event_pub.publish.assert_called_once()
        event = json.loads(self.node.event_pub.publish.call_args.args[0].data)
        self.assertEqual((event['gesture'], event['person_id'], event['arm']), ('FOLLOW',5,'RIGHT'))
        self.assertTrue(event['confirmed'])
        self.control_state = 'FOLLOW'
        self.gesture(ids=(8,))
        self.node.select_pub.publish.assert_called_once()

    def test_fresh_gesture_after_loss_rearms_without_restart(self):
        self.gesture()
        self.target()
        self.control_state = 'FOLLOW'
        self.gesture(ids=(8,))  # A bystander cannot change an active selection.
        self.node.select_pub.publish.assert_called_once()
        self.control_state = 'WAIT_COMMAND'
        self.tick()
        for _ in range(10):
            self.bodies([person(8)])  # Reappearance/static arm does not trigger.
            self.tick()
        self.node.event_pub.publish.assert_called_once()
        self.gesture(ids=(8,))
        self.target(person_id=8)
        self.assertEqual(self.node.event_pub.publish.call_count, 2)
        event = json.loads(self.node.event_pub.publish.call_args.args[0].data)
        self.assertEqual((event['gesture'], event['person_id']), ('FOLLOW', 8))

    def test_stop_stay_and_missing_permission_block_selection(self):
        for state in ('FOLLOW', 'SAFE_STOP', 'STAY_LOCKED', 'GO_TO_ASTRONAUT'):
            self.control_state = state
            self.gesture()
        self.node.select_pub.publish.assert_not_called()
        self.node.event_pub.publish.assert_not_called()
        self.control_state = 'WAIT_COMMAND'
        self.tick()
        self.guidance()
        self.auto_guidance = False
        self.tick(.6)
        self.gesture()
        self.node.select_pub.publish.assert_not_called()

    def test_replayed_guidance_cannot_rearm_or_extend_permission(self):
        old_stamp = self.wall
        self.auto_guidance = False
        self.tick(.6)
        self.guidance(published_timestamp=old_stamp)
        self.assertFalse(self.node.enabled)
        self.tick()
        self.guidance()
        old_stamp = self.wall
        self.tick()
        self.control_state = 'SAFE_STOP'
        self.guidance()
        self.guidance(state='WAIT_COMMAND', follow_gesture_allowed=True,
                      published_timestamp=old_stamp)
        self.assertFalse(self.node.enabled)

    def test_pending_confirmation_canceled_by_stop_permission(self):
        self.gesture()
        self.control_state = 'SAFE_STOP'
        self.guidance()
        self.target()
        self.node.event_pub.publish.assert_not_called()

    def test_cooldown_and_new_complete_motion_required(self):
        self.gesture()
        self.target()
        for _ in range(5):
            self.tick()
            self.bodies([person(5, -.14)])
        self.node.select_pub.publish.assert_called_once()
        self.tick(1.)
        for dy in [-.14]*6 + [.22]*6:
            self.bodies([person(5, dy)])
            self.tick()
        self.node.select_pub.publish.assert_called_once()
        self.gesture()
        self.target()
        self.assertEqual(self.node.event_pub.publish.call_count, 2)

    def test_left_arm(self):
        self.gesture(side='LEFT')
        self.target()
        event = json.loads(self.node.event_pub.publish.call_args.args[0].data)
        self.assertEqual(event['arm'], 'LEFT')

    def test_simultaneous_people_are_ambiguous(self):
        self.gesture(ids=(5, 8))
        self.node.select_pub.publish.assert_not_called()
        self.node.event_pub.publish.assert_not_called()

    def test_missing_arm_breaks_sequence(self):
        for dy in [.22]*8 + [-.14]*6:
            self.bodies([person(5,dy)])
            self.tick()
        p = person(5)
        p['keypoints_m'][3] = [None,None,None]
        self.bodies([p])
        self.tick()
        for _ in range(6):
            self.bodies([person(5)])
            self.tick()
        self.node.select_pub.publish.assert_not_called()

    def test_loss_and_id_change_break_sequence(self):
        for state in ('SEARCHING', 'OFF', 'TERMINATE', 'ABSENT', 'NEW_ID'):
            self.node.clear_attempt()
            for dy in [.22]*8+[-.14]*6:
                self.bodies([person(5,dy)])
                self.tick()
            p = person(5)
            p['tracking_state'] = state if state not in ('ABSENT','NEW_ID') else 'OK'
            if state == 'NEW_ID': p['id'] = 9
            self.bodies([] if state == 'ABSENT' else [p])
            self.tick()
            for _ in range(6):
                self.bodies([person(5)])
                self.tick()
        self.node.select_pub.publish.assert_not_called()

    def test_malformed_input_cancels_pending(self):
        for text in ('{', 'null', '{"bad":NaN}', '{"bad":Infinity}', '{"bad":1e999}',
                     '{"people":[],"people":[]}'):
            self.gesture()
            self.assertIsNotNone(self.node.pending)
            self.node.on_bodies(String(data=text))
            self.assertIsNone(self.node.pending)
            self.target()
        self.node.event_pub.publish.assert_not_called()

    def test_pending_target_disappears(self):
        self.gesture()
        self.bodies([person(3)])
        self.target()
        self.node.event_pub.publish.assert_not_called()
        self.assertIsNone(self.node.pending)

    def test_stale_raw_and_replay_cannot_extend_gesture(self):
        self.gesture()
        stamp = self.node.last_stamp
        self.tick(.8)
        self.node.watchdog()
        self.assertEqual(self.node.last_stamp, stamp)
        self.assertIsNone(self.node.pending)
        self.bodies([person(5)], stamp=stamp)
        self.target()
        self.node.event_pub.publish.assert_not_called()

    def test_callback_checks_disconnect_without_timer(self):
        self.gesture()
        self.tick(.8)
        self.target(age_sec=0, timestamp=self.wall)
        self.assertIsNone(self.node.pending)
        self.node.event_pub.publish.assert_not_called()

    def test_selection_deadline_with_continuing_raw_input(self):
        self.gesture()
        for _ in range(32):
            self.bodies([person(5)])
            self.tick()
        self.target()
        self.node.event_pub.publish.assert_not_called()
        self.assertIsNone(self.node.pending)

    def test_bad_confirmation_cannot_activate(self):
        self.gesture()
        for update in ({'timestamp': self.node.request_wall}, {'timestamp': self.wall-2},
                       {'timestamp': self.wall+1}, {'timestamp': float('inf')},
                       {'timestamp': True}, {'age_sec': .5}, {'age_sec': False},
                       {'person_id': True}, {'valid': False}, {'locked': False},
                       {'tracking_state': 'SEARCHING'}):
            self.target(**update)
        self.node.event_pub.publish.assert_not_called()
        self.target()
        self.node.event_pub.publish.assert_called_once()

    def test_only_perception_publishers(self):
        topics = {p.topic_name for p in self.node.publishers}
        self.assertEqual(topics - {'/rosout', '/parameter_events'},
                         {'/rover/vision/select', '/rover/gesture/event'})

    def test_complete_pipeline_loss_new_id_and_stop_without_restart(self):
        from rover.mission import State
        from rover.mission_control_node import FollowController
        from rover.vision import TargetTracker

        controller, tracker = FollowController(), TargetTracker()

        def clocks():
            return dict(wall=self.wall, mono=self.mono)

        controller.command('start', **clocks())
        controller.command('arrived', **clocks())
        self.node.select_pub.publish.side_effect = lambda msg: tracker.select(
            json.loads(msg.data)['person_id'])
        self.node.event_pub.publish.side_effect = lambda msg: controller.receive_gesture(
            msg.data, **clocks())

        def step(people):
            self.tick()
            self.node.on_guidance(String(data=json.dumps(controller.guidance(**clocks()))))
            raw = json.dumps(dict(timestamp=self.wall, detected=bool(people), people=people))
            self.node.on_bodies(String(data=raw))
            snapshot = json.dumps(tracker.receive(raw, **clocks()))
            controller.receive_target(snapshot, **clocks())
            self.node.on_target(String(data=snapshot))
            return controller.guidance(**clocks())

        def wave(pid):
            result = None
            for dy in [.22]*8 + [-.14]*6 + [.22]*10:
                result = step([person(pid, dy)])
            return result

        self.assertGreater(wave(5)['linear_mps'], 0.)
        self.node.event_pub.publish.assert_called_once()
        for _ in range(6):
            self.assertEqual(step([])['linear_mps'], 0.)
        for _ in range(10):
            self.assertEqual(step([person(8)])['linear_mps'], 0.)
        self.node.event_pub.publish.assert_called_once()
        self.assertGreater(wave(8)['linear_mps'], 0.)
        self.assertEqual(controller.authorized_id, 8)
        self.assertEqual(self.node.event_pub.publish.call_count, 2)
        controller.command('stop', **clocks())
        self.assertEqual(wave(5)['linear_mps'], 0.)
        self.assertEqual(controller.mission.state, State.STOP)
        self.assertEqual(self.node.event_pub.publish.call_count, 2)
