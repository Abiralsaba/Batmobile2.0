"""Opt-in, isolated ROS transport tests. Never starts a camera or hardware node.

After building/sourcing: BATMOBILE_FOLLOW_ROS_TEST=1 ROS_DOMAIN_ID=176
ROS_LOCALHOST_ONLY=1 python3 -m unittest discover -s tests -p test_follow_ros.py -v
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

from test_follow import follow_event, target

try:
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String
except ImportError:
    rclpy = None


@unittest.skipUnless(rclpy is not None and os.environ.get('BATMOBILE_FOLLOW_ROS_TEST') == '1',
                     'Opt-in ROS transport test; see module docstring')
class TestFollowROS(unittest.TestCase):
    def setUp(self):
        # A dedicated non-default domain is mandatory for synthetic publications.
        self.assertNotEqual(os.environ.get('ROS_DOMAIN_ID', '0'), '0')
        self.assertEqual(os.environ.get('ROS_LOCALHOST_ONLY'), '1')
        self.context = Context()
        rclpy.init(context=self.context)
        self.addCleanup(self.context.shutdown)
        self.node = rclpy.create_node('stage4_software_test', context=self.context)
        self.addCleanup(self.node.destroy_node)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.addCleanup(self.executor.shutdown)
        self.messages = []
        self.states = []
        self.events = []
        self.node.create_subscription(String, '/rover/follow/guidance',
                                      lambda msg: self.messages.append(json.loads(msg.data)), 100)
        self.node.create_subscription(String, '/rover/state',
                                      lambda msg: self.states.append(msg.data), 100)
        self.node.create_subscription(String, '/rover/gesture/event',
                                      lambda msg: self.events.append(json.loads(msg.data)), 10)
        self.cmd_pub = self.node.create_publisher(String, '/dev/cmd', 10)
        self.target_pub = self.node.create_publisher(String, '/rover/vision/target', 10)
        self.event_pub = self.node.create_publisher(String, '/rover/gesture/event', 10)
        self.seq = 0
        # Frozen ROS time exercises the steady-clock watchdog independently.
        self.start_node('mission_control_node', ['--ros-args', '-p', 'use_sim_time:=true'])
        self.until(lambda: self.cmd_pub.get_subscription_count() == 1
                   and self.target_pub.get_subscription_count() == 1
                   and self.event_pub.get_subscription_count() == 2
                   and self.messages, timeout=8.)

    def start_node(self, executable, args=()):
        path = Path(get_package_prefix('mars_rover')) / 'lib' / 'mars_rover' / executable
        output = tempfile.TemporaryFile(mode='w+t')
        self.addCleanup(output.close)
        process = subprocess.Popen([str(path), *args], stdout=output, stderr=output)

        def shutdown():
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    process.wait(timeout=5)
            output.seek(0)
            self.assertEqual(process.returncode, 0, output.read())

        self.addCleanup(shutdown)

    def until(self, condition, timeout=3., tick=None):
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            if tick:
                tick()
            self.executor.spin_once(timeout_sec=.02)
        self.assertTrue(condition(), f'Timed out; latest guidance: {self.messages[-1:]}')

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=.01)

    def command(self, command, state):
        previous = len(self.states)
        self.cmd_pub.publish(String(data=command))
        self.until(lambda: state in self.states[previous:])

    def publish_target(self, **updates):
        self.seq += 1
        self.target_pub.publish(String(data=json.dumps(target(time.time(), self.seq, **updates))))

    def assert_no_motor_publishers(self):
        topics = dict(self.node.get_publisher_names_and_types_by_node('mission_control', '/'))
        self.assertEqual(set(topics) - {'/rosout', '/parameter_events'},
                         {'/rover/state', '/rover/follow/guidance', '/rover/mission/start_status'})
        self.assertEqual(topics['/rover/follow/guidance'], ['std_msgs/msg/String'])
        self.assertTrue(all(msg['motors_enabled'] is False for msg in self.messages))

    def test_coordinate_prompt_reports_blocked_navigation(self):
        executable = Path(get_package_prefix('mars_rover')) / 'lib/mars_rover/start_mission'
        result = subprocess.run([str(executable)], input='23.8103\n90.4125\n',
                                capture_output=True, text=True, timeout=15.)
        self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
        self.assertIn('Destination latitude', result.stdout)
        self.assertIn('Destination longitude', result.stdout)
        self.assertIn('Mission controller received your destination', result.stdout)
        self.assertIn('Rover remains stopped', result.stdout)
        self.spin_for(.2)
        self.assertEqual(self.messages[-1]['state'], 'READY')
        self.assert_no_motor_publishers()

    def test_synthetic_topics_watchdog_and_publisher_contract(self):
        self.command('start', 'GO_TO_ASTRONAUT')
        self.command('arrived', 'WAIT_COMMAND')
        self.until(lambda: self.messages[-1]['tracking_valid'], tick=self.publish_target)
        self.event_pub.publish(String(data=json.dumps(follow_event(time.time()))))  # Exactly one FOLLOW.
        self.until(lambda: self.messages[-1]['reason'] == 'FOLLOWING', tick=self.publish_target)
        for distance, reason in ((2., 'DISTANCE_DEADBAND'), (1.5, 'TOO_CLOSE'), (3., 'FOLLOWING')):
            self.until(lambda: self.messages[-1]['reason'] == reason,
                       tick=lambda: self.publish_target(z=distance))
        self.until(lambda: self.messages[-1]['tracking_reason'] == 'ID_MISMATCH',
                   tick=lambda: self.publish_target(pid=8))
        self.assertIsNone(self.messages[-1]['person_id'])
        self.assertEqual(self.messages[-1]['linear_mps'], 0.)
        self.publish_target()
        self.spin_for(.1)
        self.assertEqual(self.messages[-1]['reason'], 'FOLLOW_GESTURE_REQUIRED')
        self.event_pub.publish(String(data=json.dumps(follow_event(time.time()))))
        self.until(lambda: self.messages[-1]['reason'] == 'FOLLOWING', tick=self.publish_target)
        # No further targets: steady timer must revoke even with no ROS /clock.
        began = time.monotonic()
        self.until(lambda: self.messages[-1]['reason'] == 'FOLLOW_GESTURE_REQUIRED', timeout=1.5)
        self.assertLess(time.monotonic() - began, 1.)
        self.assertEqual(self.messages[-1]['state'], 'WAIT_COMMAND')
        self.assertEqual((self.messages[-1]['linear_mps'], self.messages[-1]['angular_rps']), (0., 0.))
        self.publish_target()
        self.spin_for(.1)
        self.assertFalse(self.messages[-1]['reset_required'])
        self.until(lambda: self.messages[-1]['tracking_valid'], tick=self.publish_target)
        self.assertEqual(self.messages[-1]['linear_mps'], 0.)
        self.event_pub.publish(String(data=json.dumps(follow_event(time.time()))))
        self.until(lambda: self.messages[-1]['reason'] == 'FOLLOWING', tick=self.publish_target)
        self.command('stop', 'SAFE_STOP')
        self.assertEqual(self.messages[-1]['linear_mps'], 0.)
        self.assert_no_motor_publishers()

    def test_existing_gesture_and_vision_pipeline_with_synthetic_bodies(self):
        self.start_node('vision')
        self.start_node('gesture')
        bodies_pub = self.node.create_publisher(String, '/rover/vision/bodies', 1)
        self.until(lambda: bodies_pub.get_subscription_count() == 2, timeout=8.)
        self.command('start', 'GO_TO_ASTRONAUT')
        self.command('arrived', 'WAIT_COMMAND')
        # Exercise the real DOWN -> UP -> DOWN detector, target select and confirmation.
        for wrist_y in [.22]*8 + [-.14]*6 + [.22]*16:
            points = [[0., 0., 3.] for _ in range(18)]
            points[4][1] = wrist_y
            points[7][1] = .22
            person = dict(id=7, tracking_state='OK', x_m=0., y_m=0., z_m=3.,
                          distance_m=3., bearing_deg=0., keypoints_m=points)
            bodies_pub.publish(String(data=json.dumps(dict(timestamp=time.time(), detected=True,
                                                           people=[person]))))
            self.spin_for(.08)
        self.assertEqual(len(self.events), 1)
        self.assertEqual(self.events[0]['person_id'], 7)
        self.assertEqual(self.events[0]['gesture'], 'FOLLOW')
        self.assertEqual(self.messages[-1]['state'], 'FOLLOW')
        self.assertEqual(self.messages[-1]['linear_mps'], .20)
        # Lose the person, then repeat the gesture with a new tracking ID.
        # Neither vision, gesture nor mission controller is restarted.
        for _ in range(6):
            bodies_pub.publish(String(data=json.dumps(dict(timestamp=time.time(),
                                                           detected=False, people=[]))))
            self.spin_for(.08)
        self.assertEqual(self.messages[-1]['state'], 'WAIT_COMMAND')
        self.assertEqual(self.messages[-1]['linear_mps'], 0.)
        for wrist_y in [.22]*8 + [-.14]*6 + [.22]*16:
            points = [[0., 0., 3.] for _ in range(18)]
            points[4][1], points[7][1] = wrist_y, .22
            person = dict(id=9, tracking_state='OK', x_m=0., y_m=0., z_m=3.,
                          distance_m=3., bearing_deg=0., keypoints_m=points)
            bodies_pub.publish(String(data=json.dumps(dict(timestamp=time.time(), detected=True,
                                                           people=[person]))))
            self.spin_for(.08)
        self.assertEqual(len(self.events), 2)
        self.assertEqual(self.events[-1]['person_id'], 9)
        self.assertEqual(self.messages[-1]['state'], 'FOLLOW')
        self.assertEqual(self.messages[-1]['linear_mps'], .20)
        self.assert_no_motor_publishers()
