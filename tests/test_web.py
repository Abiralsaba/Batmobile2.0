import json
import tempfile
import unittest
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path

from fastapi.testclient import TestClient

from rover.web import create_app


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class DashboardApiTests(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.config_path = Path(self.temporary_directory.name) / 'input.json'
        self.config_path.write_bytes((PROJECT_ROOT / 'input.json').read_bytes())
        self.app = create_app(
            config_path=self.config_path,
            web_path=PROJECT_ROOT / 'web' / 'index.html',
        )
        self.client = TestClient(self.app)

    def tearDown(self):
        self.client.close()
        self.temporary_directory.cleanup()

    def snapshot(self):
        response = self.client.get('/api/config')
        self.assertEqual(response.status_code, 200)
        return response.json()

    def configured_target(self):
        snapshot = self.snapshot()
        data = deepcopy(snapshot['data'])
        data['astronaut_assistance']['initial_target'] = {
            'latitude': 38.406,
            'longitude': -110.792,
        }
        response = self.client.put(
            '/api/config',
            json={'etag': snapshot['etag'], 'data': data},
        )
        self.assertEqual(response.status_code, 200)
        return response.json()

    def test_dashboard_contains_every_required_screen_and_switching_hook(self):
        response = self.client.get('/')
        self.assertEqual(response.status_code, 200)
        html = response.text
        screens = {
            'home', 'gnss-approach', 'wait-command', 'follow', 'stay',
            'find-hammer', 'hammer-approach', 'pick-hammer', 'come', 'give',
            'route-start', 'route-target-1', 'route-target-2',
            'route-waypoints', 'route-status', 'recovery',
        }
        for screen in screens:
            self.assertIn(f'id="screen-{screen}"', html)
            self.assertIn(f'data-screen="{screen}"', html)
            self.assertIn(f'data-screen-target="{screen}"', html)
        self.assertIn('function showScreen(name)', html)
        self.assertIn("setInterval(() => reloadConfig(false), 2000)", html)

    def test_configuration_save_is_validated_and_atomic(self):
        snapshot = self.snapshot()
        data = deepcopy(snapshot['data'])
        data['route_finding']['start'] = {'latitude': 12.5, 'longitude': -45.25}
        response = self.client.put(
            '/api/config',
            json={'etag': snapshot['etag'], 'data': data},
        )
        self.assertEqual(response.status_code, 200)
        saved = response.json()
        self.assertNotEqual(saved['etag'], snapshot['etag'])
        self.assertEqual(saved['data']['route_finding']['start']['latitude'], 12.5)
        self.assertEqual(json.loads(self.config_path.read_text()), saved['data'])
        self.assertEqual(list(self.config_path.parent.glob('.input.*.tmp')), [])

    def test_invalid_coordinate_does_not_modify_file(self):
        snapshot = self.snapshot()
        original = self.config_path.read_bytes()
        data = deepcopy(snapshot['data'])
        data['route_finding']['target_1'] = {'latitude': 91, 'longitude': 0}
        response = self.client.put(
            '/api/config',
            json={'etag': snapshot['etag'], 'data': data},
        )
        self.assertEqual(response.status_code, 422)
        self.assertIn('between -90 and 90', response.json()['detail'])
        self.assertEqual(self.config_path.read_bytes(), original)

    def test_external_edit_is_visible_and_stale_save_conflicts(self):
        original = self.snapshot()
        external = deepcopy(original['data'])
        external['saved_parameters']['route_success_threshold_m'] = 1.75
        self.config_path.write_text(json.dumps(external, indent=2) + '\n')

        refreshed = self.snapshot()
        self.assertTrue(refreshed['valid'])
        self.assertNotEqual(refreshed['etag'], original['etag'])
        self.assertEqual(refreshed['data']['saved_parameters']['route_success_threshold_m'], 1.75)

        stale_data = deepcopy(original['data'])
        stale_data['saved_parameters']['route_success_threshold_m'] = 2.0
        response = self.client.put(
            '/api/config',
            json={'etag': original['etag'], 'data': stale_data},
        )
        self.assertEqual(response.status_code, 409)
        self.assertEqual(response.json()['current']['etag'], refreshed['etag'])
        self.assertEqual(json.loads(self.config_path.read_text())['saved_parameters']['route_success_threshold_m'], 1.75)

    def test_malformed_json_is_reported_and_blocks_commands(self):
        self.config_path.write_text('{malformed')
        response = self.client.get('/api/config')
        self.assertEqual(response.status_code, 200)
        self.assertFalse(response.json()['valid'])
        self.assertIsNone(response.json()['data'])
        command = self.client.post(
            '/api/command',
            json={'command': 'start', 'confirmed': True, 'source': 'test'},
        )
        self.assertEqual(command.status_code, 409)
        self.assertIn('malformed', command.json()['detail'])

    def test_dev_transition_never_sends_hardware_command(self):
        self.configured_target()
        unconfirmed = self.client.post(
            '/api/command',
            json={'command': 'start', 'confirmed': False, 'source': 'test'},
        )
        self.assertEqual(unconfirmed.status_code, 409)

        response = self.client.post(
            '/api/command',
            json={'command': 'start', 'confirmed': True, 'source': 'test'},
        )
        self.assertEqual(response.status_code, 200)
        body = response.json()
        self.assertEqual(body['state'], 'GO_TO_ASTRONAUT')
        self.assertFalse(body['hardware_command_sent'])
        self.assertEqual(body['status']['active_sub_mission'], 'ASTRONAUT ASSISTANCE')

    def test_follow_requires_fresh_confident_person_track(self):
        self.configured_target()
        self.client.post('/api/command', json={'command': 'start', 'confirmed': True})
        self.client.post('/api/command', json={'command': 'arrived'})

        blocked = self.client.post('/api/command', json={'command': 'follow', 'confirmed': True})
        self.assertEqual(blocked.status_code, 409)
        telemetry = self.client.post('/api/dev/telemetry', json={'updates': {
            'camera_connected': True,
            'person_detected': True,
            'person_track_id': 7,
            'person_distance_m': 3.0,
            'person_bearing_deg': 4.0,
            'person_confidence': 0.9,
        }})
        self.assertEqual(telemetry.status_code, 200)
        accepted = self.client.post('/api/command', json={'command': 'follow', 'confirmed': True})
        self.assertEqual(accepted.status_code, 200)
        self.assertEqual(accepted.json()['state'], 'FOLLOW')

    def test_live_mode_disables_motion_and_locks_unknown_motion_edits(self):
        snapshot = self.configured_target()
        live = self.client.post('/api/mode', json={'mode': 'LIVE'})
        self.assertEqual(live.status_code, 200)
        self.assertEqual(live.json()['motion_state'], 'unknown')
        self.assertFalse(live.json()['live_navigation_ready'])

        command = self.client.post('/api/command', json={'command': 'start', 'confirmed': True})
        self.assertEqual(command.status_code, 409)
        save = self.client.put(
            '/api/config',
            json={'etag': snapshot['etag'], 'data': snapshot['data']},
        )
        self.assertEqual(save.status_code, 423)

    def test_complete_dev_sequence_and_feedback_are_simulation_only(self):
        snapshot = self.configured_target()
        data = deepcopy(snapshot['data'])
        data['astronaut_assistance']['updated_astronaut_location'] = {
            'latitude': 38.406, 'longitude': -110.792,
            'timestamp': datetime.now(timezone.utc).isoformat(),
        }
        saved = self.client.put('/api/config', json={'etag': snapshot['etag'], 'data': data})
        self.assertEqual(saved.status_code, 200)

        def command(event, expected):
            result = self.client.post('/api/command', json={'command': event, 'confirmed': True})
            self.assertEqual(result.status_code, 200, result.text)
            self.assertEqual(result.json()['state'], expected)
            self.assertFalse(result.json()['hardware_command_sent'])

        command('start', 'GO_TO_ASTRONAUT')
        command('arrived', 'WAIT_COMMAND')
        premature = self.client.post('/api/command', json={'command': 'fetch', 'confirmed': True})
        self.assertEqual(premature.status_code, 409)
        self.client.post('/api/dev/telemetry', json={'updates': {
            'camera_connected': True, 'person_detected': True, 'person_confidence': 0.9,
        }})
        command('follow', 'FOLLOW')
        command('stay', 'STAY_LOCKED')
        premature = self.client.post('/api/command', json={'command': 'come', 'confirmed': True})
        self.assertEqual(premature.status_code, 409)
        command('search', 'FIND_HAMMER')
        command('found', 'WAIT_APPROVAL')
        missing_target = self.client.post('/api/command', json={'command': 'yes', 'confirmed': True})
        self.assertEqual(missing_target.status_code, 409)
        self.client.post('/api/dev/telemetry', json={'updates': {
            'hammer_detected': True, 'hammer_valid': True,
        }})
        for event, state in (
            ('yes', 'APPROACH_HAMMER'), ('near', 'FUTURE_ARM_PICKUP'),
            ('pickup_confirmed', 'WAIT_COME'), ('come', 'COME'),
            ('arrived', 'WAIT_GIVE'), ('give', 'FUTURE_ARM_RELEASE'),
            ('release_confirmed', 'MISSION_COMPLETE'),
        ):
            command(event, state)
        self.assertEqual(self.client.get('/api/status').json()['motion_state'], 'stationary')

    def test_live_mode_rejects_simulated_action_feedback(self):
        self.configured_target()
        self.client.post('/api/mode', json={'mode': 'LIVE'})
        for event in ('fetch', 'tracking_lost', 'pickup_confirmed', 'give', 'release_confirmed'):
            with self.subTest(event=event):
                result = self.client.post('/api/command', json={'command': event, 'confirmed': True})
                self.assertEqual(result.status_code, 409)
                self.assertIn('LIVE controls are disabled', result.json()['detail'])

    def test_sub_mission_selection_and_telemetry_validation(self):
        selected = self.client.post('/api/sub-mission', json={'sub_mission': 'ROUTE FINDING'})
        self.assertEqual(selected.status_code, 200)
        self.assertEqual(selected.json()['active_sub_mission'], 'ROUTE FINDING')
        rejected = self.client.post(
            '/api/dev/telemetry',
            json={'updates': {'person_distance_m': 'near'}},
        )
        self.assertEqual(rejected.status_code, 409)


if __name__ == '__main__':
    unittest.main()
