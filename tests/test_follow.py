"""Stage 4 policy tests; synthetic clocks/vision data, no ROS or camera needed."""
import json
import math
import unittest

from rover.mission import State
from rover.mission_control_node import FollowController
from rover.nav import follow, Twist2
from rover.vision import FRAME_ID, TargetTracker


def target(wall, seq=1, pid=7, x=0., z=3., **updates):
    data = dict(valid=True, locked=True, tracking_state='OK', person_id=pid,
                frame_id=FRAME_ID, timestamp=wall, published_timestamp=wall,
                observation_seq=seq, age_sec=0., reason='TRACKING_OK',
                reselection_required=False, x_m=x, y_m=0., z_m=z,
                distance_m=math.hypot(x, z))
    data.update(updates)
    return data


def follow_event(wall, pid=7, **updates):
    data = dict(gesture='FOLLOW', confirmed=True, person_id=pid, timestamp=wall)
    data.update(updates)
    return data


class TestFollowController(unittest.TestCase):
    def setUp(self):
        self.c = FollowController()
        self.wall, self.mono, self.seq = 1000., 10., 0

    def clocks(self):
        return dict(wall=self.wall, mono=self.mono)

    def tick(self, dt=.1):
        self.wall += dt
        self.mono += dt

    def command(self, name):
        return self.c.command(name, **self.clocks())

    def ready(self):
        self.assertTrue(self.command('start'))
        self.assertTrue(self.command('arrived'))

    def send_target(self, **updates):
        self.seq += 1
        data = target(self.wall, self.seq, **updates)
        self.c.receive_target(json.dumps(data), **self.clocks())
        return data

    def gesture(self, **updates):
        return self.c.receive_gesture(json.dumps(follow_event(self.wall, **updates)), **self.clocks())

    def activate(self, **updates):
        self.ready()
        self.send_target(**updates)
        self.assertTrue(self.gesture())

    def guidance(self):
        result = self.c.guidance(**self.clocks())
        json.dumps(result, allow_nan=False)
        self.assertFalse(result['motors_enabled'])
        self.assertEqual(result['calibration_status'], 'CALIBRATION_REQUIRED')
        self.assertEqual(result['source_frame'], FRAME_ID)
        return result

    def assert_stopped(self):
        result = self.guidance()
        self.assertEqual((result['linear_mps'], result['angular_rps']), (0., 0.))
        return result

    def test_confirmed_follow_uses_follow_transition_from_wait(self):
        self.activate()
        self.assertEqual(self.c.mission.state, State.FOLLOW)
        self.assertEqual(self.c.mission.history[-1], ('WAIT_COMMAND', 'follow', 'FOLLOW', 'gesture'))
        self.assertEqual(self.c.authorized_id, 7)

    def test_post_follow_mission_stages_keep_guidance_and_gestures_disabled(self):
        self.activate()
        for event, expected in (
            ('stay', State.STAY), ('fetch', State.FIND), ('found', State.ASK),
            ('yes', State.NEAR), ('near', State.ARM),
            ('pickup_confirmed', State.WAIT_COME), ('come', State.COME),
            ('arrived', State.WAIT_GIVE), ('give', State.GIVE),
            ('release_confirmed', State.DONE),
        ):
            with self.subTest(event=event):
                self.tick(.01)
                self.send_target()
                self.assertTrue(self.command(event))
                self.assertEqual(self.c.mission.state, expected)
                self.assertFalse(self.gesture())
                self.assertFalse(self.assert_stopped()['follow_gesture_allowed'])

    def test_wrong_states_ignore_follow_event(self):
        for state in State:
            if state == State.WAIT:
                continue
            with self.subTest(state=state):
                self.c = FollowController()
                self.c.mission.state = state
                self.send_target()
                self.assertFalse(self.gesture())
                self.assertEqual(self.c.mission.state, state)
                self.assertIsNone(self.c.authorized_id)
                self.assert_stopped()

    def test_target_alone_and_mock_follow_do_not_authorize_guidance(self):
        self.ready()
        self.send_target()
        self.assertEqual(self.c.mission.state, State.WAIT)
        self.assertTrue(self.command('follow'))  # Preserve mock FSM behavior.
        self.assertEqual(self.assert_stopped()['reason'], 'FOLLOW_AUTHORIZATION_REQUIRED')

    def test_repeated_follow_never_changes_authorized_id(self):
        self.activate()
        for pid in (7, 8, 7):
            self.tick()
            self.send_target()
            self.assertFalse(self.gesture(pid=pid))
            self.assertEqual(self.guidance()['person_id'], 7)
            self.assertGreater(self.guidance()['linear_mps'], 0)

    def test_event_before_matching_vision_confirmation(self):
        self.ready()
        self.assertFalse(self.gesture())
        self.assertEqual(self.c.mission.state, State.WAIT)
        self.send_target(pid=8)
        self.assertEqual(self.c.mission.state, State.WAIT)
        self.tick()
        self.send_target()
        self.assertEqual(self.c.mission.state, State.FOLLOW)

    def test_pending_event_expires_and_duplicate_cannot_refresh_it(self):
        self.ready()
        event = json.dumps(follow_event(self.wall))
        self.c.receive_gesture(event, **self.clocks())
        self.tick(.4)
        self.assertFalse(self.c.receive_gesture(event, **self.clocks()))
        self.tick(.11)
        self.send_target()
        self.assertEqual(self.c.mission.state, State.WAIT)

    def test_stale_malformed_unconfirmed_and_invalid_identity_gestures(self):
        self.ready()
        self.send_target()
        for updates in ({'confirmed': False}, {'confirmed': 1}, {'gesture': 'COME'},
                        {'gesture': 'follow'},
                        {'person_id': True}, {'person_id': -1}, {'person_id': 7.0},
                        {'person_id': None}, {'timestamp': self.wall-.51},
                        {'timestamp': self.wall+.2}, {'timestamp': True}):
            self.assertFalse(self.gesture(**updates), updates)
        for raw in ('null', '[]', '{', '{"a":NaN}', '{"a":Infinity}',
                    '{"a":1e999}', '{"a":1,"a":2}'):
            self.assertFalse(self.c.receive_gesture(raw, **self.clocks()))
        self.assertTrue(self.gesture())

    def test_one_follow_continues_at_three_meters(self):
        self.activate()
        for _ in range(30):
            self.tick()
            self.send_target()
            result = self.guidance()
            self.assertEqual(result['linear_mps'], .20)
            self.assertEqual(result['angular_rps'], 0.)
            self.assertEqual(result['distance_error_m'], 1.)
            self.assertEqual(result['reason'], 'FOLLOWING')
        self.assertEqual(sum(item[1] == 'follow' for item in self.c.mission.history), 1)

    def test_distance_deadband_boundaries_and_no_reverse(self):
        self.activate()
        for distance in (2., 2.01, 2.14, 2.15, 1.85, 1.5):
            self.tick()
            self.send_target(z=distance)
            result = self.assert_stopped()
            self.assertTrue(result['tracking_valid'])
            self.assertEqual(result['reason'], 'TOO_CLOSE' if distance < 1.85 else 'DISTANCE_DEADBAND')
        self.tick()
        self.send_target(z=2.151)
        self.assertGreater(self.guidance()['linear_mps'], 0)

    def test_camera_axis_sign_turning_and_rotate_first(self):
        self.activate()
        for x, expected_sign in ((.3, -1), (-.3, 1), (3., -1)):
            self.tick()
            self.send_target(x=x)
            result = self.guidance()
            self.assertGreater(result['angular_rps'] * expected_sign, 0)
            self.assertLessEqual(abs(result['angular_rps']), .3)
            if x == 3.:
                self.assertEqual(result['linear_mps'], 0.)
                self.assertEqual(result['reason'], 'ROTATE_FIRST')

    def test_loss_and_recovery_require_new_follow_gesture(self):
        self.activate()
        for state in ('SEARCHING', 'OFF', 'TERMINATE'):
            self.tick()
            self.send_target(valid=False, tracking_state=state, reason='TARGET_LOST')
            self.assert_stopped()
            self.assertIsNone(self.c.authorized_id)
            self.tick()
            self.send_target()
            self.assert_stopped()  # Seeing the person again is not authorization.
            self.assertTrue(self.gesture())
            self.assertGreater(self.guidance()['linear_mps'], 0)

    def test_missing_and_different_ids_stop_without_switching(self):
        self.activate()
        for pid in (None, 8, True):
            self.tick()
            self.send_target(pid=pid)
            self.assert_stopped()
            self.assertIsNone(self.c.authorized_id)
        self.tick()
        self.send_target()
        self.assert_stopped()
        self.assertTrue(self.gesture())
        self.assertGreater(self.guidance()['linear_mps'], 0)

    def test_stale_positions_and_old_publication_rejected(self):
        self.activate()
        for update in ({'timestamp': self.wall-.6}, {'age_sec': .51},
                       {'timestamp': self.wall+.2}, {'published_timestamp': self.wall-.6}):
            self.send_target(**update)
            self.assert_stopped()

    def test_cached_snapshots_cannot_extend_capture_deadline(self):
        self.activate()
        data = dict(self.c.target)
        # Wall clock frozen; only monotonic time advances. Timer heartbeat is not capture.
        for _ in range(6):
            self.mono += .1
            data['published_timestamp'] = self.wall
            self.c.receive_target(json.dumps(data), **self.clocks())
        self.assertEqual(self.assert_stopped()['tracking_reason'], 'STALE_TARGET')
        self.assertIsNone(self.c.authorized_id)

    def test_replayed_observation_cannot_recover_invalidated_target(self):
        self.activate()
        cached = json.dumps(self.c.target)
        self.send_target(valid=False, reason='TARGET_LOST')
        self.c.receive_target(cached, **self.clocks())
        self.assert_stopped()

    def test_fresh_stamp_with_repeated_sequence_is_not_fresh_capture(self):
        self.activate()
        self.tick()
        self.c.receive_target(json.dumps(target(self.wall, seq=self.c.sequence)), **self.clocks())
        self.assertEqual(self.assert_stopped()['tracking_reason'], 'STALE_TARGET')

    def test_no_messages_monotonic_watchdog_revokes(self):
        self.activate()
        self.mono += .51  # No callbacks; even a frozen wall clock cannot extend guidance.
        result = self.assert_stopped()
        self.assertFalse(result['reset_required'])
        self.assertEqual(result['state'], 'WAIT_COMMAND')
        self.assertTrue(result['follow_gesture_allowed'])
        self.assertIsNone(self.c.authorized_id)

    def test_disconnect_detected_before_next_target_without_timer(self):
        self.activate()
        self.tick(.6)
        self.send_target()
        self.assert_stopped()
        self.assertFalse(self.c.reset_required)
        self.assertFalse(self.gesture())
        self.tick()
        self.send_target()
        self.assertTrue(self.gesture())
        self.assertGreater(self.guidance()['linear_mps'], 0)

    def test_vision_disconnect_and_reselection_revoke(self):
        for reason in ('SENSOR_DISCONNECTED', 'RESELECTION_REQUIRED'):
            self.setUp()
            self.activate()
            self.tick()
            self.send_target(valid=False, reason=reason, reselection_required=True)
            self.assertEqual(self.assert_stopped()['state'], 'WAIT_COMMAND')
            self.tick()
            self.send_target()
            self.assert_stopped()
            self.assertTrue(self.gesture())
            self.assertGreater(self.guidance()['linear_mps'], 0)

    def test_tracking_loss_rejects_old_gesture_without_operator_reset(self):
        self.activate()
        old_event = json.dumps(follow_event(self.wall))
        self.tick()
        self.send_target(reason='RESELECTION_REQUIRED', valid=False)
        self.assertFalse(self.c.receive_gesture(old_event, **self.clocks()))
        self.tick()
        self.send_target()
        self.assertEqual(self.c.mission.state, State.WAIT)
        self.assertTrue(self.gesture())

    def test_stop_and_stay_override_follow(self):
        for command, expected in (('STOP', State.STOP), ('STAY', State.STAY)):
            self.setUp()
            self.activate()
            self.assertTrue(self.command(command))
            self.assertEqual(self.c.mission.state, expected)
            self.assert_stopped()
            self.tick()
            self.send_target()
            self.assertFalse(self.gesture())
            self.assert_stopped()

    def test_reset_does_not_legitimize_cached_target(self):
        self.activate()
        cached = json.dumps(self.c.target)
        self.command('stop')
        self.command('reset')
        self.ready()
        self.c.receive_target(cached, **self.clocks())
        self.assertFalse(self.gesture())
        self.assert_stopped()
        self.tick()
        # New vision session can restart its sequence only after explicit reset.
        self.c.receive_target(json.dumps(target(self.wall, seq=1)), **self.clocks())
        self.assertTrue(self.gesture())

    def test_stop_cancels_pending_follow_and_delayed_event(self):
        self.ready()
        delayed = json.dumps(follow_event(self.wall))
        self.gesture()
        self.command('stop')
        self.tick(.1)
        self.command('reset')
        self.ready()
        self.send_target()
        self.assertFalse(self.c.receive_gesture(delayed, **self.clocks()))
        self.assertEqual(self.c.mission.state, State.WAIT)

    def test_vision_sequence_restart_revokes_authorization(self):
        self.activate()
        self.tick()
        self.send_target()
        self.tick()
        self.c.receive_target(json.dumps(target(self.wall, seq=1)), **self.clocks())
        self.assertEqual(self.assert_stopped()['reason'], 'FOLLOW_GESTURE_REQUIRED')
        self.tick()
        self.c.receive_target(json.dumps(target(self.wall, seq=1, pid=9)), **self.clocks())
        self.assert_stopped()
        self.assertTrue(self.gesture(pid=9))
        self.assertEqual(self.c.authorized_id, 9)

    def test_invalid_target_numbers_frames_and_json_stop(self):
        self.activate()
        for change in ({'z_m': 0}, {'x_m': None}, {'distance_m': 8.},
                       {'observation_seq': True}, {'frame_id': 'base_link'},
                       {'valid': 1}, {'locked': False}, {'tracking_state': 'OFF'}):
            self.send_target(**change)
            self.assert_stopped()
        for raw in ('{', 'null', '[]', '{"a":NaN}', '{"a":Infinity}',
                    '{"a":1e999}', '{"a":1,"a":2}'):
            self.c.receive_target(raw, **self.clocks())
            self.assert_stopped()

    def test_real_vision_schema_and_reacquisition(self):
        self.ready()
        tracker = TargetTracker()
        tracker.select(7)

        def feed(state='OK'):
            raw = dict(timestamp=self.wall, detected=True, people=[{
                'id': 7, 'x_m': 0., 'y_m': 0., 'z_m': 3., 'distance_m': 3.,
                'bearing_deg': 0., 'tracking_state': state,
                'keypoints_m': [[0., 0., 3.]] * 18}])
            snapshot = tracker.receive(json.dumps(raw), **self.clocks())
            self.c.receive_target(json.dumps(snapshot), **self.clocks())
            self.tick()

        for _ in range(3):
            feed()
        self.assertTrue(self.gesture())
        feed('SEARCHING')
        self.assert_stopped()
        for _ in range(2):
            feed()
            self.assert_stopped()
        feed()
        self.assert_stopped()
        self.assertTrue(self.gesture())
        self.assertGreater(self.guidance()['linear_mps'], 0)

    def test_stop_after_tracking_loss_cannot_be_cleared_by_gesture(self):
        self.activate()
        self.tick()
        self.send_target(valid=False, reason='TARGET_LOST')
        self.command('stop')
        self.tick()
        self.send_target()
        self.assertFalse(self.gesture())
        self.assertEqual(self.assert_stopped()['state'], 'SAFE_STOP')
        self.assertFalse(self.guidance()['follow_gesture_allowed'])

    def test_stay_or_stop_not_changed_by_camera_disconnect(self):
        for command in ('stay', 'stop'):
            self.setUp()
            self.activate()
            self.command(command)
            expected = self.c.mission.state
            self.tick(1.)
            self.send_target(valid=False, reason='SENSOR_DISCONNECTED', reselection_required=True)
            self.tick()
            self.send_target()
            self.assertFalse(self.gesture())
            self.assertEqual(self.c.mission.state, expected)
            self.assert_stopped()

    def test_new_gesture_can_select_new_id_after_loss_but_not_before(self):
        self.activate()
        self.tick()
        self.assertFalse(self.gesture(pid=9))
        self.assertEqual(self.c.authorized_id, 7)
        self.send_target(valid=False, reason='TARGET_LOST')
        self.tick()
        self.send_target(pid=9)
        self.assert_stopped()
        self.assertFalse(self.gesture(pid=7))  # Mismatched event cannot activate.
        self.assert_stopped()
        self.tick(.51)  # Expire pending mismatched event.
        self.send_target(pid=9)
        self.assertTrue(self.gesture(pid=9))
        self.assertEqual(self.c.authorized_id, 9)


class TestFollowerDeadband(unittest.TestCase):
    def test_invalid_tolerance_is_safe(self):
        for value in (-1., float('nan'), float('inf')):
            self.assertEqual(follow(3., 0., tolerance_m=value), Twist2())

    def test_deadband_does_not_suppress_turning(self):
        twist = follow(2., .1)
        self.assertEqual(twist.linear, 0.)
        self.assertGreater(twist.angular, 0.)
