"""Minimal pure-Python tests; runnable in Ubuntu 22.04 VM without a Jetson."""
import math
import json
import unittest
from rover.mission import Mission, State
from rover.nav import follow, Twist2
from rover.safety import gate
from rover.voice import parse
from rover.vision import Person
from rover.hw import drive
from rover.nav import GNSSGoal
from rover.mission_control_node import FollowController, prompt_destination


class TestDestinationStartup(unittest.TestCase):
    def setUp(self):
        self.controller = FollowController()
        self.request = dict(request_id='operator-1', timestamp=1000.,
                            latitude=23.8103, longitude=90.4125)

    def submit(self, **changes):
        request = dict(self.request, **changes)
        return self.controller.request_destination(json.dumps(request), wall=1000.)

    def test_goal_bounds_and_zero_are_valid(self):
        for lat, lon in ((0, 0), (-90, -180), (90, 180), (23.8103, 90.4125)):
            self.assertEqual(GNSSGoal(lat, lon).longitude, lon)

    def test_prompt_retries_invalid_values(self):
        answers = iter(['', 'north', 'nan', '91', '23.8103', 'inf', '-181', '90.4125'])
        prompts, errors = [], []

        def read(prompt):
            prompts.append(prompt)
            return next(answers)

        self.assertEqual(prompt_destination(read, errors.append), GNSSGoal(23.8103, 90.4125))
        self.assertEqual(len(errors), 6)
        self.assertIn('latitude', prompts[0])
        self.assertIn('longitude', prompts[-1])

    def test_cancel_does_not_produce_a_goal(self):
        def cancelled(_):
            raise EOFError
        with self.assertRaises(EOFError):
            prompt_destination(cancelled)

    def test_destination_is_received_but_navigation_does_not_start(self):
        reply = self.submit()
        self.assertTrue(reply['destination_accepted'])
        self.assertFalse(reply['navigation_started'])
        self.assertEqual(reply['reason'], 'NAVIGATION_UNAVAILABLE')
        self.assertEqual(reply['request_id'], self.request['request_id'])
        self.assertEqual(self.controller.destination, GNSSGoal(23.8103, 90.4125))
        self.assertEqual(self.controller.mission.state, State.READY)
        self.assertEqual(self.controller.mission.history, [])
        guidance = self.controller.guidance(wall=1000., mono=10.)
        self.assertEqual((guidance['linear_mps'], guidance['angular_rps']), (0., 0.))
        self.assertFalse(guidance['motors_enabled'])
        self.assertFalse(guidance['follow_gesture_allowed'])

    def test_bad_coordinates_cannot_replace_destination(self):
        self.submit()
        original = self.controller.destination
        for field in ('latitude', 'longitude'):
            for value in (None, True, '23.8', [], 181, -181, 10 ** 300,
                          float('nan'), float('inf')):
                with self.subTest(field=field, value=value):
                    reply = self.submit(**{field: value})
                    self.assertFalse(reply['destination_accepted'])
                    self.assertFalse(reply['navigation_started'])
                    self.assertEqual(self.controller.destination, original)

    def test_stale_future_and_replayed_requests_rejected(self):
        for stamp in (994.9, 1000.2, True, None):
            self.assertFalse(self.submit(timestamp=stamp)['destination_accepted'])
        self.assertTrue(self.submit()['destination_accepted'])
        self.assertEqual(self.submit()['reason'], 'STALE_OR_REPLAYED_REQUEST')

    def test_all_non_ready_states_reject_destination_changes(self):
        for state in State:
            if state == State.READY:
                continue
            with self.subTest(state=state):
                self.controller.mission.state = state
                reply = self.submit()
                self.assertEqual(reply['reason'], 'MISSION_NOT_READY')
                self.assertIsNone(self.controller.destination)
                self.assertEqual(self.controller.mission.state, state)

    def test_malformed_and_unknown_fields_rejected(self):
        for text in ('null', '[]', '{}', 'bad json',
                     '{"request_id":"a","request_id":"b"}'):
            reply = self.controller.request_destination(text, wall=1000.)
            self.assertFalse(reply['destination_accepted'])
        self.assertFalse(self.submit(extra=True)['destination_accepted'])

    def test_stop_clears_destination_and_reset_does_not_reuse_it(self):
        self.submit()
        for event in ('stop', 'reset'):
            self.controller.command(event, wall=1000., mono=10.)
            self.assertIsNone(self.controller.destination)
        self.assertFalse(self.submit()['destination_accepted'])


class TestMission(unittest.TestCase):
    def test_start_arrived(self):
        m = Mission()
        self.assertTrue(m.event('start'))
        self.assertEqual(m.state, State.GO)
        self.assertTrue(m.event('arrived'))
        self.assertEqual(m.state, State.WAIT)

    def test_stay_locked(self):
        m = Mission()
        for event in ('start', 'arrived', 'follow', 'stay'):
            self.assertTrue(m.event(event))
        self.assertFalse(m.event('follow'))
        self.assertEqual(m.state, State.STAY)
        self.assertFalse(m.event('come'))
        self.assertTrue(m.event('fetch'))
        self.assertEqual(m.state, State.FIND)

    def test_stop_overrides(self):
        m = Mission()
        self.assertTrue(m.event('stop'))
        self.assertEqual(m.state, State.STOP)
        self.assertFalse(m.event('start'))
        self.assertTrue(m.event('reset'))
        self.assertEqual(m.state, State.READY)

    def test_yes_only_in_approval_state(self):
        m = Mission()
        self.assertFalse(m.event('yes'))
        for name in ('start','arrived','follow','stay','fetch','found'):
            self.assertTrue(m.event(name))
        self.assertEqual(m.state, State.ASK)
        self.assertTrue(m.event('yes'))
        self.assertEqual(m.state, State.NEAR)
        self.assertFalse(m.event('yes'))

    def test_complete_sequence_requires_action_feedback(self):
        m = Mission()
        steps = (
            ('start', State.GO), ('arrived', State.WAIT),
            ('follow', State.FOLLOW), ('stay', State.STAY),
            ('fetch', State.FIND), ('found', State.ASK),
            ('yes', State.NEAR), ('near', State.ARM),
            ('pickup_confirmed', State.WAIT_COME), ('come', State.COME),
            ('arrived', State.WAIT_GIVE), ('give', State.GIVE),
            ('release_confirmed', State.DONE),
        )
        for event, expected in steps:
            with self.subTest(event=event):
                self.assertTrue(m.event(event, source='mock_feedback'))
                self.assertEqual(m.state, expected)
                history = list(m.history)
                self.assertFalse(m.event(event))  # duplicate cannot advance
                self.assertEqual(m.history, history)
        self.assertFalse(m.event('start'))
        self.assertFalse(m.event('follow'))

    def test_come_give_and_feedback_cannot_skip_stages(self):
        required = {'come': State.WAIT_COME, 'give': State.WAIT_GIVE,
                    'pickup_confirmed': State.ARM, 'release_confirmed': State.GIVE,
                    'fetch': State.STAY, 'follow': State.WAIT}
        for event, allowed in required.items():
            for state in State:
                with self.subTest(event=event, state=state):
                    m = Mission()
                    m.state = state
                    self.assertEqual(m.event(event), state == allowed)
                    if state != allowed:
                        self.assertEqual(m.state, state)
                        self.assertEqual(m.history, [])

    def test_tracking_loss_needs_new_follow_and_stay_latches(self):
        m = Mission()
        for event in ('start', 'arrived', 'follow', 'tracking_lost'):
            self.assertTrue(m.event(event))
        self.assertEqual(m.state, State.WAIT)
        self.assertFalse(m.event('fetch'))
        self.assertTrue(m.event('follow'))
        # Reaching following distance is not completion of the FOLLOW task.
        self.assertFalse(m.event('arrived'))
        self.assertTrue(m.event('stay'))
        self.assertFalse(m.event('tracking_lost'))
        self.assertFalse(m.event('follow'))
        self.assertEqual(m.state, State.STAY)

    def test_cancel_hammer_approval_returns_to_stay_for_retry(self):
        m = Mission()
        for event in ('start', 'arrived', 'follow', 'stay', 'fetch', 'found', 'no'):
            self.assertTrue(m.event(event))
        self.assertEqual(m.state, State.STAY)
        self.assertFalse(m.event('come'))
        self.assertTrue(m.event('fetch'))

    def test_stop_overrides_every_stage_and_requires_reset(self):
        for state in State:
            with self.subTest(state=state):
                m = Mission()
                m.state = state
                self.assertTrue(m.event('stop'))
                self.assertEqual(m.state, State.STOP)
                for event in ('follow', 'fetch', 'come', 'give', 'arrived',
                              'pickup_confirmed', 'release_confirmed'):
                    self.assertFalse(m.event(event))
                self.assertTrue(m.event('reset'))
                self.assertEqual(m.state, State.READY)

    def test_unrecognized_event(self):
        m = Mission()
        self.assertFalse(m.event('teleport'))
        self.assertEqual(m.state, State.READY)


class TestVoice(unittest.TestCase):
    def test_allowlist(self):
        self.assertEqual(parse('  FOLLOW! '), 'follow')
        self.assertEqual(parse('Emergency Stop'), 'stop')
        self.assertEqual(parse('GIVE'), 'give')
        self.assertIsNone(parse('pickup_confirmed'))
        self.assertIsNone(parse('release_confirmed'))
        self.assertIsNone(parse('follow then drive fast'))
        self.assertIsNone(parse('yes go to the hammer'))


class TestNavSafety(unittest.TestCase):
    def test_turn_directions(self):
        self.assertGreater(follow(3, 1).angular, 0)
        self.assertLess(follow(3, -1).angular, 0)

    def test_invalid_position(self):
        self.assertEqual(follow(float('nan'), 2), Twist2())
        self.assertEqual(follow(-1, 0), Twist2())

    def test_gate_blocks_bad_depth(self):
        proposed = Twist2(.1, .2)
        self.assertEqual(gate(proposed, allowed=True, fresh_person=True,
                              depth_ok=False, clear_path=True, estop_ok=True), Twist2())

    def test_gate_all_clear(self):
        proposed = Twist2(.1, .2)
        self.assertEqual(gate(proposed, allowed=True, fresh_person=True,
                              depth_ok=True, clear_path=True, estop_ok=True), proposed)

    def test_gate_blocks_nan(self):
        proposed = Twist2(float('nan'), .2)
        self.assertEqual(gate(proposed, allowed=True, fresh_person=True,
                              depth_ok=True, clear_path=True, estop_ok=True), Twist2())

    def test_motor_disabled(self):
        with self.assertRaises(RuntimeError):
            drive(0.1, 0.0)

    def test_valid_person(self):
        self.assertTrue(Person(1, 3, .5, 10.2, .9).valid())
        self.assertFalse(Person(1, 3, .5, 10.2, float('nan')).valid())


if __name__ == '__main__':
    unittest.main()
