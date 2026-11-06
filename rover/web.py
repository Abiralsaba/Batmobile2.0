"""Offline FastAPI dashboard for safe Autonomous Mission control. DEV/ROS-ready.

Input: validated browser actions and input.json. Output: UI/API state only.
No endpoint publishes motor commands; LIVE motion remains disabled.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import tempfile
import threading
import time
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, JSONResponse
from pydantic import BaseModel, Field

from rover.mission import Mission, State


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUT_PATH = PACKAGE_ROOT / 'input.json'
DEFAULT_WEB_PATH = PACKAGE_ROOT / 'web' / 'index.html'


class ConfigurationError(ValueError):
    """The saved mission configuration is malformed or unsafe."""


class ConfigurationConflict(RuntimeError):
    """The configuration changed after the browser loaded it."""

    def __init__(self, snapshot: dict[str, Any]):
        super().__init__('input.json changed; reload before saving')
        self.snapshot = snapshot


class EditingBlocked(RuntimeError):
    """LIVE configuration cannot be edited unless stationary is confirmed."""


class SafetyRejected(RuntimeError):
    """A requested transition failed a safety precondition."""


class SaveConfigurationRequest(BaseModel):
    etag: str = Field(min_length=1, max_length=128)
    data: dict[str, Any]


class CommandRequest(BaseModel):
    command: str = Field(min_length=1, max_length=32)
    confirmed: bool = False
    source: str = Field(default='dashboard', min_length=1, max_length=32)


class ModeRequest(BaseModel):
    mode: str = Field(min_length=3, max_length=4)


class SubMissionRequest(BaseModel):
    sub_mission: str = Field(min_length=4, max_length=32)


class TelemetryRequest(BaseModel):
    updates: dict[str, Any]


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _expect_object(value: Any, path: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ConfigurationError(f'{path} must be an object')
    return value


def _expect_exact_keys(value: dict[str, Any], keys: set[str], path: str) -> None:
    missing = keys - set(value)
    extra = set(value) - keys
    if missing:
        raise ConfigurationError(f'{path} is missing: {", ".join(sorted(missing))}')
    if extra:
        raise ConfigurationError(f'{path} has unsupported keys: {", ".join(sorted(extra))}')


def _validate_coordinate(value: Any, path: str, *, allow_null: bool = True) -> dict[str, Any]:
    coordinate = _expect_object(value, path)
    _expect_exact_keys(coordinate, {'latitude', 'longitude'}, path)
    latitude = coordinate['latitude']
    longitude = coordinate['longitude']
    if latitude is None and longitude is None and allow_null:
        return {'latitude': None, 'longitude': None}
    if latitude is None or longitude is None:
        raise ConfigurationError(f'{path} latitude and longitude must both be null or both be numbers')
    if not _is_number(latitude) or not -90 <= float(latitude) <= 90:
        raise ConfigurationError(f'{path}.latitude must be between -90 and 90')
    if not _is_number(longitude) or not -180 <= float(longitude) <= 180:
        raise ConfigurationError(f'{path}.longitude must be between -180 and 180')
    return {'latitude': float(latitude), 'longitude': float(longitude)}


def _validate_timestamp(value: Any, path: str) -> str | None:
    if value is None:
        return None
    if not isinstance(value, str) or len(value) > 64:
        raise ConfigurationError(f'{path} must be an ISO-8601 string or null')
    try:
        parsed = datetime.fromisoformat(value.replace('Z', '+00:00'))
    except ValueError as exc:
        raise ConfigurationError(f'{path} must be a valid ISO-8601 timestamp') from exc
    if parsed.tzinfo is None:
        raise ConfigurationError(f'{path} must include a timezone')
    return parsed.astimezone(timezone.utc).isoformat().replace('+00:00', 'Z')


def _validate_named_points(value: Any, path: str) -> list[dict[str, Any]]:
    if not isinstance(value, list) or len(value) > 100:
        raise ConfigurationError(f'{path} must be a list with at most 100 points')
    result = []
    for index, item in enumerate(value):
        item_path = f'{path}[{index}]'
        point = _expect_object(item, item_path)
        _expect_exact_keys(point, {'name', 'latitude', 'longitude'}, item_path)
        name = point['name']
        if not isinstance(name, str) or not name.strip() or len(name.strip()) > 64:
            raise ConfigurationError(f'{item_path}.name must be 1-64 characters')
        coordinate = _validate_coordinate(
            {'latitude': point['latitude'], 'longitude': point['longitude']},
            item_path,
            allow_null=False,
        )
        result.append({'name': name.strip(), **coordinate})
    return result


def _validate_zones(value: Any, path: str) -> list[dict[str, Any]]:
    if not isinstance(value, list) or len(value) > 50:
        raise ConfigurationError(f'{path} must be a list with at most 50 zones')
    result = []
    for index, item in enumerate(value):
        item_path = f'{path}[{index}]'
        zone = _expect_object(item, item_path)
        _expect_exact_keys(zone, {'name', 'latitude', 'longitude', 'radius_m'}, item_path)
        name = zone['name']
        if not isinstance(name, str) or not name.strip() or len(name.strip()) > 64:
            raise ConfigurationError(f'{item_path}.name must be 1-64 characters')
        coordinate = _validate_coordinate(
            {'latitude': zone['latitude'], 'longitude': zone['longitude']},
            item_path,
            allow_null=False,
        )
        radius = zone['radius_m']
        if not _is_number(radius) or not 0 < float(radius) <= 10000:
            raise ConfigurationError(f'{item_path}.radius_m must be greater than 0 and at most 10000')
        result.append({'name': name.strip(), **coordinate, 'radius_m': float(radius)})
    return result


def _positive_parameter(parameters: dict[str, Any], name: str, maximum: float) -> float:
    value = parameters[name]
    if not _is_number(value) or not 0 < float(value) <= maximum:
        raise ConfigurationError(f'saved_parameters.{name} must be greater than 0 and at most {maximum:g}')
    return float(value)


def validate_configuration(value: Any) -> dict[str, Any]:
    """Validate and normalize the complete input.json document."""
    root = _expect_object(value, 'input.json')
    _expect_exact_keys(
        root,
        {'schema_version', 'astronaut_assistance', 'route_finding', 'saved_parameters'},
        'input.json',
    )
    if root['schema_version'] != 1:
        raise ConfigurationError('schema_version must be 1')

    astronaut = _expect_object(root['astronaut_assistance'], 'astronaut_assistance')
    _expect_exact_keys(
        astronaut,
        {'initial_target', 'updated_astronaut_location', 'intermediate_waypoints'},
        'astronaut_assistance',
    )
    initial_target = _validate_coordinate(astronaut['initial_target'], 'astronaut_assistance.initial_target')
    updated_raw = _expect_object(
        astronaut['updated_astronaut_location'],
        'astronaut_assistance.updated_astronaut_location',
    )
    _expect_exact_keys(
        updated_raw,
        {'latitude', 'longitude', 'timestamp'},
        'astronaut_assistance.updated_astronaut_location',
    )
    updated_coordinate = _validate_coordinate(
        {'latitude': updated_raw['latitude'], 'longitude': updated_raw['longitude']},
        'astronaut_assistance.updated_astronaut_location',
    )
    timestamp = _validate_timestamp(
        updated_raw['timestamp'],
        'astronaut_assistance.updated_astronaut_location.timestamp',
    )
    if updated_coordinate['latitude'] is None and timestamp is not None:
        raise ConfigurationError('updated astronaut timestamp must be null when coordinates are null')

    route = _expect_object(root['route_finding'], 'route_finding')
    _expect_exact_keys(route, {'start', 'target_1', 'target_2', 'waypoints', 'stay_out_zones'}, 'route_finding')

    parameters = _expect_object(root['saved_parameters'], 'saved_parameters')
    parameter_names = {
        'desired_follow_distance_m',
        'astronaut_success_threshold_m',
        'route_success_threshold_m',
        'approval_timeout_s',
        'gnss_freshness_s',
        'hammer_staging_distance_m',
    }
    _expect_exact_keys(parameters, parameter_names, 'saved_parameters')

    return {
        'schema_version': 1,
        'astronaut_assistance': {
            'initial_target': initial_target,
            'updated_astronaut_location': {
                **updated_coordinate,
                'timestamp': timestamp,
            },
            'intermediate_waypoints': _validate_named_points(
                astronaut['intermediate_waypoints'],
                'astronaut_assistance.intermediate_waypoints',
            ),
        },
        'route_finding': {
            'start': _validate_coordinate(route['start'], 'route_finding.start'),
            'target_1': _validate_coordinate(route['target_1'], 'route_finding.target_1'),
            'target_2': _validate_coordinate(route['target_2'], 'route_finding.target_2'),
            'waypoints': _validate_named_points(route['waypoints'], 'route_finding.waypoints'),
            'stay_out_zones': _validate_zones(route['stay_out_zones'], 'route_finding.stay_out_zones'),
        },
        'saved_parameters': {
            'desired_follow_distance_m': _positive_parameter(parameters, 'desired_follow_distance_m', 20),
            'astronaut_success_threshold_m': _positive_parameter(parameters, 'astronaut_success_threshold_m', 20),
            'route_success_threshold_m': _positive_parameter(parameters, 'route_success_threshold_m', 20),
            'approval_timeout_s': _positive_parameter(parameters, 'approval_timeout_s', 300),
            'gnss_freshness_s': _positive_parameter(parameters, 'gnss_freshness_s', 3600),
            'hammer_staging_distance_m': _positive_parameter(parameters, 'hammer_staging_distance_m', 20),
        },
    }


class ConfigurationStore:
    """Thread-safe, conflict-aware, atomic input.json storage."""

    def __init__(self, path: Path):
        self.path = path.resolve()
        self._lock = threading.RLock()
        if not self.path.exists():
            raise FileNotFoundError(f'configuration file not found: {self.path}')

    @staticmethod
    def _etag(raw: bytes) -> str:
        return hashlib.sha256(raw).hexdigest()

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            try:
                raw = self.path.read_bytes()
            except OSError as exc:
                return {'valid': False, 'etag': '', 'modified_ns': None, 'data': None, 'error': str(exc)}
            etag = self._etag(raw)
            try:
                decoded = json.loads(raw.decode('utf-8'))
                data = validate_configuration(decoded)
            except (UnicodeDecodeError, json.JSONDecodeError, ConfigurationError) as exc:
                return {
                    'valid': False,
                    'etag': etag,
                    'modified_ns': self.path.stat().st_mtime_ns,
                    'data': None,
                    'error': str(exc),
                }
            return {
                'valid': True,
                'etag': etag,
                'modified_ns': self.path.stat().st_mtime_ns,
                'data': data,
                'error': None,
            }

    def save(self, expected_etag: str, value: Any, *, runtime: 'DashboardRuntime') -> dict[str, Any]:
        with self._lock:
            current = self.snapshot()
            if expected_etag != current['etag']:
                raise ConfigurationConflict(current)
            if runtime.mode == 'LIVE' and runtime.motion_state != 'stationary':
                raise EditingBlocked('LIVE configuration changes require confirmed stationary motion state')
            normalized = validate_configuration(value)
            serialized = (json.dumps(normalized, indent=2, sort_keys=False) + '\n').encode('utf-8')
            temporary_name: str | None = None
            try:
                with tempfile.NamedTemporaryFile(
                    mode='wb',
                    dir=self.path.parent,
                    prefix='.input.',
                    suffix='.tmp',
                    delete=False,
                ) as temporary:
                    temporary_name = temporary.name
                    temporary.write(serialized)
                    temporary.flush()
                    os.fsync(temporary.fileno())
                os.replace(temporary_name, self.path)
                temporary_name = None
            finally:
                if temporary_name:
                    try:
                        Path(temporary_name).unlink()
                    except FileNotFoundError:
                        pass
            return self.snapshot()


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z')


def _age_seconds(timestamp: str | None) -> float | None:
    if timestamp is None:
        return None
    parsed = datetime.fromisoformat(timestamp.replace('Z', '+00:00'))
    return max(0.0, (datetime.now(timezone.utc) - parsed.astimezone(timezone.utc)).total_seconds())


class DashboardRuntime:
    """DEV simulator state; it never publishes ROS or motor commands."""

    MOVEMENT_EVENTS = {'start', 'follow', 'fetch', 'search', 'come', 'yes', 'give'}
    ALLOWED_EVENTS = {
        'start', 'arrived', 'follow', 'stay', 'come', 'fetch', 'search',
        'found', 'yes', 'no', 'near', 'stop', 'reset',
        'tracking_lost', 'pickup_confirmed', 'give', 'release_confirmed',
    }

    def __init__(self):
        self._lock = threading.RLock()
        self.mission = Mission()
        self.mode = 'DEV'
        self.active_sub_mission = 'NONE'
        self.motion_state = 'stationary'
        self.started_monotonic = time.monotonic()
        self.stay_started_monotonic: float | None = None
        self.last_command: dict[str, Any] | None = None
        self.gnss = {'fix': False, 'latitude': None, 'longitude': None, 'stamp': None}
        self.camera = {'connected': False, 'status': 'DISCONNECTED'}
        self.person = {
            'detected': False,
            'track_id': None,
            'distance_m': None,
            'bearing_deg': None,
            'confidence': None,
            'stamp': None,
            'gesture': None,
            'voice_command': None,
            'command_source': None,
        }
        self.hammer = {
            'detected': False,
            'valid': False,
            'x_m': None,
            'y_m': None,
            'z_m': None,
            'distance_m': None,
            'confidence': None,
            'stamp': None,
        }
        self.visual_reacquired = False
        self.faults: list[dict[str, str]] = []

    def set_mode(self, mode: str) -> None:
        normalized = mode.strip().upper()
        if normalized not in {'DEV', 'LIVE'}:
            raise SafetyRejected('mode must be DEV or LIVE')
        with self._lock:
            self.mode = normalized
            if normalized == 'LIVE':
                self.motion_state = 'unknown'
                self.faults = [{
                    'code': 'LIVE_BACKEND_UNAVAILABLE',
                    'message': 'Real ROS navigation and motor integration are not connected.',
                }]
            else:
                self.motion_state = 'stationary'
                self.faults = []

    def set_sub_mission(self, sub_mission: str) -> None:
        normalized = sub_mission.strip().upper().replace('_', ' ')
        if normalized not in {'NONE', 'ASTRONAUT ASSISTANCE', 'ROUTE FINDING'}:
            raise SafetyRejected('sub_mission must be NONE, ASTRONAUT ASSISTANCE, or ROUTE FINDING')
        with self._lock:
            self.active_sub_mission = normalized

    def _fresh_person(self, max_age_s: float = 0.5) -> bool:
        age = _age_seconds(self.person['stamp'])
        return bool(
            self.person['detected']
            and self.camera['connected']
            and self.person['confidence'] is not None
            and self.person['confidence'] >= 0.65
            and age is not None
            and age <= max_age_s
        )

    def _fresh_hammer(self, max_age_s: float = 5.0) -> bool:
        age = _age_seconds(self.hammer['stamp'])
        return bool(self.hammer['detected'] and self.hammer['valid'] and age is not None and age <= max_age_s)

    def apply_telemetry(self, updates: dict[str, Any]) -> None:
        """Apply allowlisted DEV simulator fields; never accepts code or ROS topics."""
        with self._lock:
            if self.mode != 'DEV':
                raise SafetyRejected('DEV telemetry cannot be changed in LIVE mode')
            allowed = {
                'motion_state', 'gnss_fix', 'gnss_latitude', 'gnss_longitude',
                'camera_connected', 'person_detected', 'person_track_id',
                'person_distance_m', 'person_bearing_deg', 'person_confidence',
                'gesture', 'voice_command', 'command_source', 'hammer_detected',
                'hammer_valid', 'hammer_x_m', 'hammer_y_m', 'hammer_z_m',
                'hammer_distance_m', 'hammer_confidence', 'visual_reacquired',
            }
            extra = set(updates) - allowed
            if extra:
                raise SafetyRejected(f'unsupported telemetry fields: {", ".join(sorted(extra))}')
            boolean_fields = {
                'gnss_fix', 'camera_connected', 'person_detected',
                'hammer_detected', 'hammer_valid', 'visual_reacquired',
            }
            for name in boolean_fields & set(updates):
                if not isinstance(updates[name], bool):
                    raise SafetyRejected(f'{name} must be true or false')
            numeric_fields = {
                'gnss_latitude', 'gnss_longitude', 'person_distance_m',
                'person_bearing_deg', 'person_confidence', 'hammer_x_m',
                'hammer_y_m', 'hammer_z_m', 'hammer_distance_m',
                'hammer_confidence',
            }
            for name in numeric_fields & set(updates):
                if updates[name] is not None and not _is_number(updates[name]):
                    raise SafetyRejected(f'{name} must be a finite number or null')
            if 'person_track_id' in updates:
                track_id = updates['person_track_id']
                if track_id is not None and (not isinstance(track_id, int) or isinstance(track_id, bool)):
                    raise SafetyRejected('person_track_id must be an integer or null')
            text_fields = {'gesture', 'voice_command', 'command_source'}
            for name in text_fields & set(updates):
                value = updates[name]
                if value is not None and (not isinstance(value, str) or len(value) > 128):
                    raise SafetyRejected(f'{name} must be a string of at most 128 characters or null')
            for name in {'person_distance_m', 'hammer_distance_m'} & set(updates):
                if updates[name] is not None and float(updates[name]) < 0:
                    raise SafetyRejected(f'{name} cannot be negative')
            for name in {'person_confidence', 'hammer_confidence'} & set(updates):
                if updates[name] is not None and not 0 <= float(updates[name]) <= 1:
                    raise SafetyRejected(f'{name} must be between 0 and 1')
            if 'motion_state' in updates:
                motion = updates['motion_state']
                if motion not in {'stationary', 'moving', 'unknown'}:
                    raise SafetyRejected('motion_state must be stationary, moving, or unknown')
                self.motion_state = motion
            if 'gnss_fix' in updates:
                self.gnss['fix'] = updates['gnss_fix']
                self.gnss['stamp'] = _utc_now()
            for source, target in (('gnss_latitude', 'latitude'), ('gnss_longitude', 'longitude')):
                if source in updates:
                    value = updates[source]
                    if value is not None and not _is_number(value):
                        raise SafetyRejected(f'{source} must be a finite number or null')
                    self.gnss[target] = None if value is None else float(value)
            if self.gnss['latitude'] is not None and not -90 <= self.gnss['latitude'] <= 90:
                raise SafetyRejected('gnss_latitude must be between -90 and 90')
            if self.gnss['longitude'] is not None and not -180 <= self.gnss['longitude'] <= 180:
                raise SafetyRejected('gnss_longitude must be between -180 and 180')
            if 'camera_connected' in updates:
                connected = updates['camera_connected']
                self.camera = {'connected': connected, 'status': 'DEV CAMERA' if connected else 'DISCONNECTED'}
            person_map = {
                'person_detected': 'detected', 'person_track_id': 'track_id',
                'person_distance_m': 'distance_m', 'person_bearing_deg': 'bearing_deg',
                'person_confidence': 'confidence', 'gesture': 'gesture',
                'voice_command': 'voice_command', 'command_source': 'command_source',
            }
            person_changed = False
            for source, target in person_map.items():
                if source in updates:
                    self.person[target] = updates[source]
                    person_changed = True
            if person_changed:
                self.person['stamp'] = _utc_now()
            hammer_map = {
                'hammer_detected': 'detected', 'hammer_valid': 'valid',
                'hammer_x_m': 'x_m', 'hammer_y_m': 'y_m', 'hammer_z_m': 'z_m',
                'hammer_distance_m': 'distance_m', 'hammer_confidence': 'confidence',
            }
            hammer_changed = False
            for source, target in hammer_map.items():
                if source in updates:
                    self.hammer[target] = updates[source]
                    hammer_changed = True
            if hammer_changed:
                self.hammer['stamp'] = _utc_now()
            if 'visual_reacquired' in updates:
                self.visual_reacquired = updates['visual_reacquired']

    def command(self, request: CommandRequest, configuration: dict[str, Any]) -> dict[str, Any]:
        event = request.command.strip().lower()
        if event not in self.ALLOWED_EVENTS:
            raise SafetyRejected('command is not allowlisted')
        with self._lock:
            if self.mode == 'LIVE' and event != 'stop':
                raise SafetyRejected('LIVE controls are disabled until the real ROS navigation backend is verified')
            if event in self.MOVEMENT_EVENTS and not request.confirmed:
                raise SafetyRejected('movement-capable transitions require explicit confirmation')

            if event == 'start':
                self.active_sub_mission = 'ASTRONAUT ASSISTANCE'
                target = configuration['astronaut_assistance']['initial_target']
                if target['latitude'] is None:
                    raise SafetyRejected('save a valid astronaut GNSS target before START')
            elif event == 'follow' and not self._fresh_person():
                raise SafetyRejected('FOLLOW requires a fresh, confident DEV person track')
            elif event == 'come':
                target = configuration['astronaut_assistance']['updated_astronaut_location']
                age = _age_seconds(target['timestamp'])
                freshness = configuration['saved_parameters']['gnss_freshness_s']
                if target['latitude'] is None or age is None or age > freshness:
                    raise SafetyRejected('COME requires fresh saved astronaut coordinates')
            elif event == 'yes' and not self._fresh_hammer():
                raise SafetyRejected('YES requires a fresh valid hammer target')

            actual_event = 'fetch' if event == 'search' and self.mission.state == State.STAY else event
            if event == 'search' and self.mission.state == State.FIND:
                accepted = True
            else:
                accepted = self.mission.event(actual_event, source=request.source)
            if not accepted:
                raise SafetyRejected(f'{event} is not valid from {self.mission.state.value}')

            if self.mission.state == State.STAY:
                self.stay_started_monotonic = time.monotonic()
            elif event != 'stay':
                self.stay_started_monotonic = None
            if event == 'stop':
                self.motion_state = 'stationary' if self.mode == 'DEV' else 'unknown'
            elif event in self.MOVEMENT_EVENTS:
                self.motion_state = 'moving' if self.mode == 'DEV' else 'unknown'
            elif event in {'arrived', 'stay', 'near', 'no', 'reset', 'tracking_lost',
                           'found', 'pickup_confirmed', 'release_confirmed'}:
                self.motion_state = 'stationary'

            self.last_command = {
                'command': event,
                'source': request.source,
                'timestamp': _utc_now(),
                'accepted': True,
            }
            return {'accepted': True, 'state': self.mission.state.value, 'hardware_command_sent': False}

    def status(self, configuration: dict[str, Any] | None) -> dict[str, Any]:
        with self._lock:
            parameters = (configuration or {}).get('saved_parameters', {})
            desired_distance = parameters.get('desired_follow_distance_m', 2.0)
            direction = 'STOP'
            if self.mission.state == State.FOLLOW and self._fresh_person():
                bearing = float(self.person['bearing_deg'] or 0)
                distance = float(self.person['distance_m'] or 0)
                if abs(bearing) > 20:
                    direction = 'LEFT' if bearing > 0 else 'RIGHT'
                elif distance > desired_distance + 0.25:
                    direction = 'FORWARD'
            stay_seconds = 0
            if self.stay_started_monotonic is not None:
                stay_seconds = int(time.monotonic() - self.stay_started_monotonic)
            return {
                'mode': self.mode,
                'active_sub_mission': self.active_sub_mission,
                'state': self.mission.state.value,
                'motion_state': self.motion_state,
                'connection': 'DEV SIMULATOR' if self.mode == 'DEV' else 'ROS DISCONNECTED',
                'ros_connected': False,
                'mission_control_status': 'SAFE DEV' if self.mode == 'DEV' else 'LIVE BLOCKED',
                'gnss': deepcopy(self.gnss),
                'camera': deepcopy(self.camera),
                'person': deepcopy(self.person),
                'hammer': deepcopy(self.hammer),
                'visual_reacquired': self.visual_reacquired,
                'movement_direction': direction,
                'desired_follow_distance_m': desired_distance,
                'stay_seconds': stay_seconds,
                'last_command': deepcopy(self.last_command),
                'faults': deepcopy(self.faults),
                'software_stop_note': 'Software STOP is not a physical emergency stop.',
                'live_navigation_ready': False,
                'arm_ready': False,
                'uptime_s': int(time.monotonic() - self.started_monotonic),
            }


def _resolve_web_path() -> Path:
    if DEFAULT_WEB_PATH.exists():
        return DEFAULT_WEB_PATH
    try:
        from ament_index_python.packages import get_package_share_directory

        candidate = Path(get_package_share_directory('mars_rover')) / 'web' / 'index.html'
        if candidate.exists():
            return candidate
    except (ImportError, LookupError):
        pass
    raise FileNotFoundError('web/index.html could not be located')


def _resolve_input_path() -> Path:
    candidates = (DEFAULT_INPUT_PATH, Path.cwd() / 'input.json')
    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()
    try:
        from ament_index_python.packages import get_package_share_directory

        candidate = Path(get_package_share_directory('mars_rover')) / 'input.json'
        if candidate.exists():
            return candidate.resolve()
    except (ImportError, LookupError):
        pass
    raise FileNotFoundError('input.json could not be located')


def create_app(config_path: Path | str | None = None, web_path: Path | str | None = None) -> FastAPI:
    store = ConfigurationStore(Path(config_path).resolve() if config_path else _resolve_input_path())
    runtime = DashboardRuntime()
    index_path = Path(web_path).resolve() if web_path else _resolve_web_path()
    app = FastAPI(title='Mars Rover Autonomous Mission Dashboard', version='0.1.0')
    app.state.store = store
    app.state.runtime = runtime

    @app.middleware('http')
    async def no_cache(request, call_next):
        response = await call_next(request)
        response.headers['Cache-Control'] = 'no-store'
        response.headers['X-Content-Type-Options'] = 'nosniff'
        return response

    @app.get('/', include_in_schema=False)
    def dashboard():
        return FileResponse(index_path, media_type='text/html')

    @app.get('/api/health')
    def health():
        return {'ok': True, 'service': 'mission_dashboard', 'mode': runtime.mode}

    @app.get('/api/config')
    def get_configuration():
        return store.snapshot()

    @app.put('/api/config')
    def save_configuration(request: SaveConfigurationRequest):
        try:
            return store.save(request.etag, request.data, runtime=runtime)
        except ConfigurationConflict as exc:
            return JSONResponse(
                status_code=409,
                content={'detail': 'input.json changed; reload before saving', 'current': exc.snapshot},
            )
        except EditingBlocked as exc:
            raise HTTPException(status_code=423, detail=str(exc)) from exc
        except ConfigurationError as exc:
            raise HTTPException(status_code=422, detail=str(exc)) from exc

    @app.get('/api/status')
    def get_status():
        snapshot = store.snapshot()
        return runtime.status(snapshot['data'] if snapshot['valid'] else None)

    @app.post('/api/mode')
    def set_mode(request: ModeRequest):
        try:
            runtime.set_mode(request.mode)
        except SafetyRejected as exc:
            raise HTTPException(status_code=422, detail=str(exc)) from exc
        return runtime.status(store.snapshot().get('data'))

    @app.post('/api/sub-mission')
    def set_sub_mission(request: SubMissionRequest):
        try:
            runtime.set_sub_mission(request.sub_mission)
        except SafetyRejected as exc:
            raise HTTPException(status_code=422, detail=str(exc)) from exc
        snapshot = store.snapshot()
        return runtime.status(snapshot['data'] if snapshot['valid'] else None)

    @app.post('/api/command')
    def command(request: CommandRequest):
        snapshot = store.snapshot()
        if not snapshot['valid']:
            raise HTTPException(status_code=409, detail='input.json is malformed; commands are blocked')
        try:
            result = runtime.command(request, snapshot['data'])
        except SafetyRejected as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return {**result, 'status': runtime.status(snapshot['data'])}

    @app.post('/api/dev/telemetry')
    def dev_telemetry(request: TelemetryRequest):
        try:
            runtime.apply_telemetry(request.updates)
        except SafetyRejected as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        snapshot = store.snapshot()
        return runtime.status(snapshot['data'] if snapshot['valid'] else None)

    return app


app = create_app()


def main() -> None:
    parser = argparse.ArgumentParser(description='Run the offline Autonomous Mission dashboard.')
    parser.add_argument('--host', default='127.0.0.1', help='Bind host (default: 127.0.0.1)')
    parser.add_argument('--port', default=8765, type=int, help='Bind port (default: 8765)')
    parser.add_argument('--input', type=Path, default=None, help='Path to input.json (default: project/package file)')
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error('--port must be between 1 and 65535')

    import uvicorn

    uvicorn.run(create_app(args.input), host=args.host, port=args.port, log_level='info')


if __name__ == '__main__':
    main()
