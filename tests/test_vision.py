"""Hardware-free tests of the exact validation/locking core used by ROS."""
import json
import math
import unittest

from rover.vision import (BODY_18, FRAME_ID, InvalidObservation, TargetTracker,
                          VisionConfig, skeleton_quality, validate_message)


def person(pid=49, x=0.1, z=2.5, state='OK'):
    return {'id': pid, 'x_m': x, 'y_m': -0.1, 'z_m': z,
            'distance_m': math.hypot(x, z),
            'bearing_deg': math.degrees(math.atan2(x, z)),
            'tracking_state': state, 'keypoints_m': [[x, 0.0, z] for _ in range(18)]}


class TestVision(unittest.TestCase):
    def setUp(self):
        self.t = TargetTracker()
        self.wall = 1000.0
        self.mono = 10.0

    def tick(self, dt=1/15):
        self.wall += dt
        self.mono += dt

    def send(self, people=None, *, timestamp=None, text=None):
        if text is None:
            people = [person()] if people is None else people
            text = json.dumps({'timestamp': self.wall if timestamp is None else timestamp,
                               'detected': bool(people), 'people': people})
        result = self.t.receive(text, wall=self.wall, mono=self.mono)
        json.dumps(result, allow_nan=False)
        return result

    def snapshot(self):
        return self.t.snapshot(wall=self.wall, mono=self.mono)

    def acquire(self, people=None):
        for _ in range(3):
            state = self.send(people)
            self.tick()
        self.assertTrue(state['valid'], state)
        return state

    def test_consecutive_acquisition_and_camera_convention(self):
        self.assertEqual(self.send()['reason'], 'ACQUIRING')
        self.tick()
        self.assertFalse(self.send()['locked'])
        self.tick()
        state = self.send()
        self.assertTrue(state['valid'])
        self.assertEqual(state['person_id'], 49)
        self.assertEqual(state['frame_id'], FRAME_ID)
        self.assertGreater(state['bearing_deg'], 0)
        self.assertAlmostEqual(state['distance_m'], math.hypot(state['x_m'], state['z_m']))
        self.assertEqual(state['valid_joints'], 18)

    def test_bad_json_immediately_invalidates_lock(self):
        self.acquire()
        for text in ('{', 'null', '[]', '{}', '{"timestamp":1,"timestamp":2}'):
            state = self.send(text=text)
            self.assertFalse(state['valid'])
            self.assertTrue(state['locked'])
            self.assertEqual(state['reason'], 'INVALID_MESSAGE')

    def test_nonfinite_and_overflow_anywhere(self):
        for token in ('NaN', 'Infinity', '-Infinity', '1e999'):
            with self.subTest(token=token):
                text = json.dumps({'timestamp': self.wall, 'detected': True, 'people': [person()]})
                text = text[:-1] + ',"extra":' + token + '}'
                self.assertEqual(self.send(text=text)['reason'], 'INVALID_MESSAGE')

    def test_null_joints_preserved_but_missing_array_rejected(self):
        p = person()
        p['keypoints_m'][4] = [None, None, None]
        p['keypoints_m'][7] = [0, None, 2.5]
        state = self.acquire([p])
        self.assertEqual(state['valid_joints'], 16)
        self.assertEqual(state['valid_joint_fraction'], 16/18)
        self.assertTrue(state['shoulders_valid'])
        self.assertTrue(state['elbows_valid'])
        self.assertFalse(state['wrists_valid'])
        self.assertEqual(state['keypoints_m'][7], [0, None, 2.5])
        del p['keypoints_m']
        self.assertEqual(self.send([p])['reason'], 'INVALID_KEYPOINTS')

    def test_official_joint_sides(self):
        self.assertEqual(BODY_18[2:8], ('RIGHT_SHOULDER', 'RIGHT_ELBOW', 'RIGHT_WRIST',
                                       'LEFT_SHOULDER', 'LEFT_ELBOW', 'LEFT_WRIST'))
        for index, label in ((2, 'right_shoulder'), (3, 'right_elbow'), (4, 'right_wrist'),
                             (5, 'left_shoulder'), (6, 'left_elbow'), (7, 'left_wrist')):
            points = [[None]*3 for _ in range(18)]
            points[index] = [0, 0, 1]
            quality = skeleton_quality(points)
            self.assertTrue(quality[label + '_valid'])
            self.assertEqual(quality['valid_joints'], 1)

    def test_searching_off_and_empty_preserve_lock(self):
        self.acquire()
        for people in ([person(state='SEARCHING')], [person(state='OFF')],
                       [person(state='TERMINATE')], []):
            state = self.send(people)
            self.assertFalse(state['valid'])
            self.assertEqual(state['reason'], 'TARGET_LOST')
            self.assertEqual(state['person_id'], 49)
            self.assertIsNone(state['distance_m'])
            self.tick()

    def test_never_acquire_predicted_states(self):
        for status in ('SEARCHING', 'OFF', 'TERMINATE'):
            for _ in range(6):
                self.assertFalse(self.send([person(state=status)])['locked'])
                self.tick()

    def test_disconnection_watchdog_and_restart_reselection(self):
        self.acquire()
        self.tick(0.51)
        state = self.snapshot()
        self.assertEqual(state['reason'], 'SENSOR_DISCONNECTED')
        self.assertFalse(state['valid'])
        self.assertEqual(state['person_id'], 49)
        for _ in range(5):
            self.assertEqual(self.send()['reason'], 'RESELECTION_REQUIRED')
            self.tick()
        self.t.select(49)
        self.acquire()

    def test_disconnect_detected_without_timer_callback(self):
        self.acquire()
        self.tick(0.6)
        self.assertEqual(self.send()['reason'], 'RESELECTION_REQUIRED')

    def test_identity_change_never_switches(self):
        self.acquire()
        for _ in range(6):
            state = self.send([person(pid=50)])
            self.assertEqual(state['person_id'], 49)
            self.assertFalse(state['valid'])
            self.tick()

    def test_multiple_people_require_selection(self):
        people = [person(), person(pid=50, z=1.0)]
        for _ in range(4):
            state = self.send(people)
            self.assertEqual(state['reason'], 'AMBIGUOUS_TARGET')
            self.assertFalse(state['locked'])
            self.tick()
        self.t.select(49)
        self.acquire(people)

    def test_spike_rejected_and_history_not_polluted(self):
        self.acquire()
        state = self.send([person(z=12)])
        self.assertFalse(state['valid'])
        self.assertEqual(state['reason'], 'INVALID_POSITION')
        self.tick()
        state = self.acquire()
        self.assertAlmostEqual(state['distance_m'], math.hypot(.1, 2.5))

    def test_median_and_optional_ema(self):
        self.t = TargetTracker(VisionConfig(ema_alpha=0.5))
        for z in (2.5, 2.6, 2.4, 2.7, 2.5):
            state = self.send([person(z=z)])
            self.tick()
        self.assertTrue(state['valid'])
        self.assertLess(abs(state['distance_m'] - math.hypot(.1, 2.5)), .05)

    def test_old_future_and_replayed_timestamps(self):
        self.acquire()
        for stamp in (self.wall-2, self.wall+2, self.t.watermark):
            state = self.send(timestamp=stamp)
            self.assertFalse(state['valid'])
            self.assertEqual(state['reason'], 'STALE')
        # A rejected future message must not poison the timestamp watermark.
        self.acquire()

    def test_repeats_do_not_advance_acquisition_or_sequence(self):
        self.send()
        stamp = self.wall
        for _ in range(6):
            self.tick()
            state = self.send(timestamp=stamp)
            self.assertFalse(state['valid'])
            self.assertEqual(state['observation_seq'], 1)

    def test_reset_and_reselection_clear_old_filter(self):
        self.acquire()
        self.t.select()
        self.assertFalse(self.snapshot()['locked'])
        state = self.acquire([person(pid=77, x=-1, z=8)])
        self.assertEqual(state['person_id'], 77)
        self.assertAlmostEqual(state['distance_m'], math.hypot(1, 8))
        self.assertLess(state['bearing_deg'], 0)
        self.t.select(99)
        state = self.acquire([person(pid=99, z=1)])
        self.assertLess(state['distance_m'], 1.1)

    def test_reset_cannot_make_old_messages_fresh(self):
        self.acquire()
        stamp = self.t.watermark
        self.t.select()
        self.assertEqual(self.send(timestamp=stamp)['reason'], 'STALE')

    def test_temporary_loss_requires_consecutive_reacquisition(self):
        self.acquire()
        self.send([])
        self.tick()
        self.assertFalse(self.send()['valid'])
        self.tick()
        self.assertFalse(self.send()['valid'])
        self.tick()
        self.assertTrue(self.send()['valid'])

    def test_all_required_fields_and_bad_types(self):
        for key in person():
            p = person()
            del p[key]
            self.assertFalse(self.send([p])['valid'])
        for key, value in (('id', True), ('id', -1), ('x_m', True), ('z_m', -1),
                           ('z_m', 0), ('z_m', 100), ('tracking_state', 'UNKNOWN'),
                           ('distance_m', -1), ('bearing_deg', 180)):
            p = person()
            p[key] = value
            self.assertFalse(self.send([p])['valid'])
        self.assertFalse(self.send([person(), person()])['valid'])

    def test_invalid_keypoint_shapes(self):
        for points in ([], [[0, 0, 1]]*17, [[0, 0, 1]]*19, [[0, 1]]*18,
                       [[False, 0, 1]]*18, [[0, 'bad', 1]]*18):
            p = person()
            p['keypoints_m'] = points
            with self.assertRaises(InvalidObservation):
                validate_message(json.dumps({'timestamp': 1000, 'detected': True,
                                             'people': [p]}), VisionConfig())

    def test_timer_republication_has_same_observation_identity_and_increasing_age(self):
        state = self.acquire()
        self.tick(.1)
        repeated = self.snapshot()
        self.assertEqual(state['timestamp'], repeated['timestamp'])
        self.assertEqual(state['observation_seq'], repeated['observation_seq'])
        self.assertGreater(repeated['age_sec'], state['age_sec'])

    def test_monotonic_timeout_survives_wall_clock_rollback(self):
        self.acquire()
        self.wall -= 100
        self.mono += .6
        self.assertFalse(self.snapshot()['valid'])

    def test_capture_age_expires_before_receive_timeout(self):
        for _ in range(3):
            state = self.send(timestamp=self.wall - .4)
            self.tick(.01)
        self.assertTrue(state['valid'])
        self.tick(.12)
        self.assertEqual(self.snapshot()['reason'], 'STALE')

    def test_broken_frame_interrupts_consecutive_acquisition(self):
        self.send()
        self.tick()
        self.send(text='{')
        self.tick()
        self.assertFalse(self.send()['valid'])
        self.tick()
        self.assertFalse(self.send()['valid'])

    def test_threshold_configuration_and_validation(self):
        self.t = TargetTracker(VisionConfig(acquisition_frames=2, reacquisition_frames=4))
        self.assertFalse(self.send()['valid'])
        self.tick()
        self.assertTrue(self.send()['valid'])
        self.tick()
        self.assertTrue(self.send()['valid'])
        for kwargs in ({'tracking_timeout_sec': 0}, {'ema_alpha': 2},
                       {'median_window': 2.5}, {'publish_hz': .1}, {'max_speed_mps': float('nan')}):
            with self.assertRaises(ValueError):
                VisionConfig(**kwargs)


if __name__ == '__main__':
    unittest.main()
