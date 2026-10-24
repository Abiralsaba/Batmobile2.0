"""Perception-only BODY_18 target validation, locking and ROS watchdog.

Live positions are meters in the ZED CAMERA/IMAGE frame (right, down, forward).
No base_link transform or actuator interface is provided. Person below remains
the legacy mock interface; it is deliberately separate from live observations.
"""
from collections import deque
from dataclasses import dataclass
import json
from math import atan2, degrees, hypot, isfinite, sqrt
from statistics import median
import time


@dataclass(frozen=True)
class Person:
    track_id: int
    x_m: float  # ROS base_link: forward
    y_m: float  # ROS base_link: left
    stamp_s: float
    confidence: float

    def valid(self) -> bool:
        return (self.track_id >= 0
                and all(isfinite(v) for v in (self.x_m, self.y_m, self.stamp_s, self.confidence))
                and self.x_m > 0 and 0 <= self.confidence <= 1)


def read_zed_person():
    """Legacy mock adapter; live camera observations use /rover/vision/target."""
    raise NotImplementedError('No calibrated camera-to-base_link transform is available')


# Verified against installed ZED SDK 5.2.3 sl.BODY_18_PARTS and official ordering:
# https://docs.stereolabs.com/docs/development/zed-sdk/modules/body-tracking
BODY_18 = (
    'NOSE', 'NECK', 'RIGHT_SHOULDER', 'RIGHT_ELBOW', 'RIGHT_WRIST',
    'LEFT_SHOULDER', 'LEFT_ELBOW', 'LEFT_WRIST', 'RIGHT_HIP', 'RIGHT_KNEE',
    'RIGHT_ANKLE', 'LEFT_HIP', 'LEFT_KNEE', 'LEFT_ANKLE', 'RIGHT_EYE',
    'LEFT_EYE', 'RIGHT_EAR', 'LEFT_EAR',
)
# Explicit local frame convention, not a claim that a TF broadcaster exists.
FRAME_ID = 'zed_left_camera_frame_optical'


@dataclass(frozen=True)
class VisionConfig:
    acquisition_frames: int = 3
    reacquisition_frames: int = 3
    tracking_timeout_sec: float = 0.5
    max_message_age_sec: float = 0.5
    future_tolerance_sec: float = 0.1
    min_forward_m: float = 0.1
    max_range_m: float = 20.0
    median_window: int = 5
    ema_alpha: float = 1.0  # 1 disables EMA; median filtering remains enabled.
    jump_margin_m: float = 0.5
    max_speed_mps: float = 3.0
    publish_hz: float = 10.0

    def __post_init__(self):
        for name in ('acquisition_frames', 'reacquisition_frames', 'median_window'):
            value = getattr(self, name)
            if type(value) is not int or not 1 <= value <= 100:
                raise ValueError(f'{name} must be an integer in [1, 100]')
        for name in self.__dataclass_fields__:
            value = getattr(self, name)
            if type(value) not in (int, float) or not isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if self.ema_alpha > 1 or self.min_forward_m >= self.max_range_m:
            raise ValueError('Invalid smoothing or distance limits')
        if self.publish_hz > 100 or self.publish_hz * self.tracking_timeout_sec < 1:
            raise ValueError('Watchdog period must not exceed tracking timeout')


class InvalidObservation(ValueError):
    def __init__(self, reason):
        super().__init__(reason)
        self.reason = reason


def strict_json(text):
    def reject_constant(value):
        raise InvalidObservation('INVALID_MESSAGE')

    def unique_object(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise InvalidObservation('INVALID_MESSAGE')
            result[key] = value
        return result

    def finite_tree(value):
        if isinstance(value, float) and not isfinite(value):
            raise InvalidObservation('INVALID_MESSAGE')
        if isinstance(value, dict):
            for item in value.values():
                finite_tree(item)
        elif isinstance(value, list):
            for item in value:
                finite_tree(item)

    try:
        result = json.loads(text, parse_constant=reject_constant,
                            object_pairs_hook=unique_object)
        finite_tree(result)  # Also rejects numeric overflow such as 1e999.
        return result
    except (ValueError, TypeError, RecursionError) as exc:
        raise InvalidObservation('INVALID_MESSAGE') from exc


def number(value):
    return type(value) in (int, float) and -1e100 < value < 1e100 and isfinite(value)


def validate_message(text, config):
    data = strict_json(text)
    if (not isinstance(data, dict) or not number(data.get('timestamp'))
            or data['timestamp'] <= 0 or type(data.get('detected')) is not bool
            or not isinstance(data.get('people'), list)
            or data['detected'] != bool(data['people'])):
        raise InvalidObservation('INVALID_MESSAGE')
    ids = set()
    for person in data['people']:
        if (not isinstance(person, dict) or type(person.get('id')) is not int
                or person['id'] < 0 or person['id'] in ids):
            raise InvalidObservation('INVALID_MESSAGE')
        ids.add(person['id'])
        # TERMINATE is also emitted by SDK 5.2.3 when retiring a track.
        if person.get('tracking_state') not in ('OK', 'SEARCHING', 'OFF', 'TERMINATE'):
            raise InvalidObservation('INVALID_TRACKING_STATE')
        if not all(number(person.get(k)) for k in
                   ('x_m', 'y_m', 'z_m', 'distance_m', 'bearing_deg')):
            raise InvalidObservation('INVALID_POSITION')
        x, y, z = (person[k] for k in ('x_m', 'y_m', 'z_m'))
        distance = hypot(x, z)
        if (not config.min_forward_m <= z <= config.max_range_m
                or distance > config.max_range_m or abs(y) > config.max_range_m
                or abs(person['distance_m'] - distance) > 0.02
                or abs(person['bearing_deg'] - degrees(atan2(x, z))) > 0.1):
            raise InvalidObservation('INVALID_POSITION')
        joints = person.get('keypoints_m')
        if not isinstance(joints, list) or len(joints) != 18:
            raise InvalidObservation('INVALID_KEYPOINTS')
        for joint in joints:
            if (not isinstance(joint, list) or len(joint) != 3
                    or any(v is not None and not number(v) for v in joint)):
                raise InvalidObservation('INVALID_KEYPOINTS')
    return data


def skeleton_quality(joints):
    valid = [all(v is not None for v in joint) for joint in joints]
    quality = {'valid_joints': sum(valid), 'total_joints': 18,
               'valid_joint_fraction': sum(valid) / 18}
    for name, right, left in (('shoulder', 2, 5), ('elbow', 3, 6), ('wrist', 4, 7)):
        quality[f'right_{name}_valid'] = valid[right]
        quality[f'left_{name}_valid'] = valid[left]
        quality[f'{name}s_valid'] = valid[right] and valid[left]
    return quality


class TargetTracker:
    """Pure deterministic core. Clocks are supplied by ROS adapter or tests.

    Every bad frame invalidates immediately. Recovery needs consecutive OK frames.
    An input gap > timeout latches RESELECTION_REQUIRED for an existing identity,
    since this wire protocol has no camera-session UUID and IDs can be reused.
    """
    def __init__(self, config=None):
        self.config = config or VisionConfig()
        self.locked_id = None
        self.candidate_id = None
        self.last_receive = None
        self.watermark = None
        self.observed_at = None
        self.observed_receive = None
        self.received_age = 0.0
        self.observation_seq = 0
        self.reselection_required = False
        self.valid = False
        self.reason = 'NO_TARGET'
        self.tracking_state = None
        self.person = None
        self.count = 0
        self._clear_filter()

    def _clear_filter(self):
        self.samples = deque(maxlen=self.config.median_window)
        self.smoothed = None
        self.last_position = None
        self.last_position_time = None

    def _invalidate(self, reason, state=None):
        self.valid = False
        self.reason = reason
        self.tracking_state = state
        self.count = 0
        self.samples.clear()
        self.smoothed = None

    def select(self, person_id=None):
        """Explicit operator reset (None) or selection; never uses cached frames."""
        if person_id is not None and (type(person_id) is not int or person_id < 0):
            raise ValueError('person_id must be a nonnegative integer')
        self.locked_id = person_id
        self.candidate_id = person_id
        self.reselection_required = False
        self._invalidate('NO_TARGET' if person_id is None else 'ACQUIRING')
        self._clear_filter()
        self.person = None
        self.observed_at = self.observed_receive = None
        # Keep watermark and sequence: resetting must not make replays acceptable.

    def receive(self, text, *, wall, mono):
        self.snapshot(wall=wall, mono=mono)  # Detect gaps even between timer ticks.
        self.last_receive = mono
        try:
            data = validate_message(text, self.config)
            stamp = data['timestamp']
            if (wall - stamp > self.config.max_message_age_sec
                    or stamp - wall > self.config.future_tolerance_sec
                    or (self.watermark is not None and stamp <= self.watermark)):
                raise InvalidObservation('STALE')
            self.watermark = stamp
        except InvalidObservation as exc:
            self._invalidate(exc.reason)
            return self.snapshot(wall=wall, mono=mono)

        if self.reselection_required:
            self._invalidate('RESELECTION_REQUIRED')
            return self.snapshot(wall=wall, mono=mono)
        people = data['people']
        if self.locked_id is None:
            if len(people) != 1:
                self.candidate_id = None
                self._clear_filter()
                self._invalidate('AMBIGUOUS_TARGET' if people else 'NO_TARGET')
                return self.snapshot(wall=wall, mono=mono)
            person = people[0]
            if person['id'] != self.candidate_id:
                self.candidate_id = person['id']
                self._clear_filter()
                self.count = 0
        else:
            person = next((p for p in people if p['id'] == self.locked_id), None)
            if person is None:
                self._invalidate('TARGET_LOST')
                return self.snapshot(wall=wall, mono=mono)
        if person['tracking_state'] != 'OK':
            self._invalidate('TARGET_LOST', person['tracking_state'])
            return self.snapshot(wall=wall, mono=mono)

        position = tuple(person[k] for k in ('x_m', 'y_m', 'z_m'))
        if self.last_position is not None:
            dt = min(max(0, stamp - self.last_position_time), self.config.tracking_timeout_sec)
            jump = sqrt(sum((a - b)**2 for a, b in zip(position, self.last_position)))
            if jump > self.config.jump_margin_m + self.config.max_speed_mps * dt:
                self._invalidate('INVALID_POSITION', 'OK')
                return self.snapshot(wall=wall, mono=mono)
        self.last_position, self.last_position_time = position, stamp
        self.samples.append((hypot(position[0], position[2]), position[0]))
        filtered = tuple(median(s[i] for s in self.samples) for i in range(2))
        if self.smoothed is not None:
            a = self.config.ema_alpha
            filtered = tuple(a * new + (1 - a) * old
                             for new, old in zip(filtered, self.smoothed))
        self.smoothed = filtered
        distance, x = filtered
        z = sqrt(max(0, distance * distance - x * x))
        self.person = dict(person, x_m=x, z_m=z, distance_m=distance,
                           bearing_deg=degrees(atan2(x, z)))
        self.observed_at, self.observed_receive = stamp, mono
        self.received_age = max(0, wall - stamp)
        self.observation_seq += 1
        self.count += 1
        required = (self.config.acquisition_frames if self.locked_id is None
                    else self.config.reacquisition_frames)
        self.valid = self.valid or self.count >= required
        if self.valid:
            self.locked_id = person['id']
        self.reason = 'TRACKING_OK' if self.valid else 'ACQUIRING'
        self.tracking_state = 'OK'
        return self.snapshot(wall=wall, mono=mono)

    def snapshot(self, *, wall, mono):
        age = None
        if self.observed_at is not None:
            age = max(0, wall - self.observed_at,
                      self.received_age + mono - self.observed_receive)
        if (self.last_receive is not None
                and mono - self.last_receive > self.config.tracking_timeout_sec):
            self.reselection_required |= self.locked_id is not None
            self._invalidate('SENSOR_DISCONNECTED')
        elif self.last_receive is None:
            self._invalidate('SENSOR_DISCONNECTED')
        elif self.valid and age is not None and age > self.config.tracking_timeout_sec:
            self._invalidate('STALE')
        elif self.count and age is not None and age > self.config.tracking_timeout_sec:
            self._invalidate('STALE')
        result = {
            'locked': self.locked_id is not None, 'valid': self.valid,
            'person_id': self.locked_id, 'tracking_state': self.tracking_state,
            'frame_id': FRAME_ID, 'timestamp': self.observed_at,
            'published_timestamp': wall, 'observation_seq': self.observation_seq,
            'age_sec': age, 'reason': self.reason,
            'reselection_required': self.reselection_required,
            'x_m': None, 'y_m': None, 'z_m': None,
            'distance_m': None, 'bearing_deg': None, 'keypoints_m': None,
            'valid_joints': 0, 'total_joints': 18, 'valid_joint_fraction': 0.0,
        }
        result.update(skeleton_quality([[None] * 3 for _ in range(18)]))
        if self.valid:
            result.update({k: self.person[k] for k in
                           ('x_m', 'y_m', 'z_m', 'distance_m', 'bearing_deg', 'keypoints_m')})
            result.update(skeleton_quality(self.person['keypoints_m']))
        return result


def main(args=None):
    import rclpy
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String

    class VisionNode(Node):
        def __init__(self):
            super().__init__('vision')
            defaults = VisionConfig()
            values = {}
            for name in defaults.__dataclass_fields__:
                values[name] = self.declare_parameter(
                    name, getattr(defaults, name),
                    ParameterDescriptor(read_only=True)).value
            self.tracker = TargetTracker(VisionConfig(**values))
            self.pub = self.create_publisher(String, '/rover/vision/target', 1)
            # Depth one prevents processing an old reliable backlog; compatible
            # with both reliable and best-effort body publishers.
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(String, '/rover/vision/bodies', self.on_bodies, qos)
            self.create_subscription(String, '/rover/vision/select', self.on_select, 1)
            self.watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.create_timer(1 / self.tracker.config.publish_hz, self.publish,
                              clock=self.watchdog_clock)
            self.get_logger().info('Perception only; CAMERA/IMAGE meters; no TF to base_link. '
                                   'Disconnect requires explicit reset/selection.')

        def publish(self, state=None):
            if state is None:
                state = self.tracker.snapshot(wall=time.time(), mono=time.monotonic())
            msg = String()
            msg.data = json.dumps(state, allow_nan=False)
            self.pub.publish(msg)

        def on_bodies(self, msg):
            state = self.tracker.receive(msg.data, wall=time.time(), mono=time.monotonic())
            if not state['valid']:
                self.publish(state)  # Loss must not wait for the 10 Hz timer.

        def on_select(self, msg):
            try:
                command = strict_json(msg.data)
                if command == {'action': 'reset'}:
                    self.tracker.select()
                elif (isinstance(command, dict) and set(command) == {'action', 'person_id'}
                      and command['action'] == 'select'):
                    if command['person_id'] is None:
                        raise ValueError('select requires an integer ID')
                    self.tracker.select(command['person_id'])
                else:
                    raise ValueError('expected reset or select with person_id')
            except ValueError as exc:
                self.get_logger().warning(f'Rejected operator command: {exc}')
                return
            self.publish()

    rclpy.init(args=args)
    node = None
    try:
        node = VisionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
