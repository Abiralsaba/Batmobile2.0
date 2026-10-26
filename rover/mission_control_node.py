"""Existing mission controller plus motor-disabled, 2 m FOLLOW diagnostics.

COME authorizes the existing WAIT_COMMAND -> follow transition for one vision
ID. CAMERA/IMAGE Z -> forward, -X -> left is approximate: calibration required.
Only String state/guidance publishers exist; this is not a motor controller.
"""
import json
from math import hypot
import time

from rover.mission import Mission, State
from rover.nav import follow
from rover.vision import FRAME_ID, number, strict_json


class FollowController:
    """Testable policy around the existing Mission; no ROS or hardware access.

    Supplied monotonic times enforce both message and capture-age watchdogs.
    Equal cached snapshots never renew an observation's deadline or recover it.
    Disconnect/reselection latches SAFE_STOP until the operator resets.
    """
    timeout = 0.5
    desired_m = 2.0
    tolerance_m = 0.15

    def __init__(self):
        self.mission = Mission()
        self.authorized_id = None
        self.pending = None
        self.event_watermark = None
        self.wait_since = None
        self.target = None
        self.target_deadline = None
        self.last_receive = None
        self.sequence = None
        self.capture_stamp = None
        self.reason = 'NO_TARGET'
        self.reset_required = False

    def _invalidate(self, reason):
        self.target = None
        self.target_deadline = None
        self.reason = reason

    def _revoke(self, reason):
        self._invalidate(reason)
        if self.authorized_id is not None or self.pending is not None:
            self.authorized_id = None
            self.pending = None
            self.reset_required = True
            self.mission.event('stop', source='tracking_watchdog')

    def _watchdog(self, *, wall, mono):
        if self.last_receive is not None and mono - self.last_receive > self.timeout:
            self._revoke('SENSOR_DISCONNECTED')
        elif (self.target is not None
              and (mono > self.target_deadline
                   or wall - self.target['timestamp'] > self.timeout
                   or self.target['timestamp'] - wall > 0.1)):
            self._invalidate('STALE_TARGET')
        if self.pending is not None:
            event, deadline = self.pending
            if (mono > deadline or wall - event['timestamp'] > self.timeout
                    or event['timestamp'] - wall > 0.1):
                self.pending = None

    def command(self, text, *, wall, mono):
        self._watchdog(wall=wall, mono=mono)
        previous = self.mission.state
        accepted = self.mission.event(text, source='mock_c2')
        if not accepted:
            return False
        self.pending = None
        if self.mission.state != State.FOLLOW:
            self.authorized_id = None
        if self.mission.state == State.WAIT:
            self.wait_since = wall
        if previous == State.STOP and self.mission.state == State.READY:
            self.reset_required = False
            self._invalidate('NO_TARGET')
            self.last_receive = self.sequence = None
            # A restarted vision process may reset sequence, but old captures and
            # COME events must remain ineligible even across an operator reset.
        return True

    def receive_gesture(self, text, *, wall, mono):
        self._watchdog(wall=wall, mono=mono)
        try:
            event = strict_json(text)
            if (not isinstance(event, dict) or event.get('gesture') != 'COME'
                    or event.get('confirmed') is not True
                    or type(event.get('person_id')) is not int or event['person_id'] < 0
                    or not number(event.get('timestamp')) or event['timestamp'] <= 0):
                return False
        except ValueError:
            return False
        stamp = event['timestamp']
        if (not -0.1 <= wall - stamp <= self.timeout
                or (self.event_watermark is not None and stamp <= self.event_watermark)):
            return False
        self.event_watermark = stamp
        if (self.reset_required or self.mission.state != State.WAIT
                or self.wait_since is None or stamp < self.wait_since
                or self.pending is not None):
            return False
        # Different ROS topics can be delivered out of order. Wait at most the
        # remaining event freshness window for vision's matching confirmation.
        self.pending = (event, mono + self.timeout - max(0, wall - stamp))
        return self._authorize()

    def _authorize(self):
        if (self.pending is None or self.target is None or self.reset_required
                or self.mission.state != State.WAIT):
            return False
        event, _ = self.pending
        if self.target['person_id'] != event['person_id']:
            return False
        if self.mission.event('follow', source='gesture'):
            self.authorized_id = event['person_id']
            self.pending = None
            return True
        return False

    def receive_target(self, text, *, wall, mono):
        self._watchdog(wall=wall, mono=mono)  # Detect gaps between timer callbacks.
        self.last_receive = mono
        try:
            data = strict_json(text)
            if not isinstance(data, dict):
                raise ValueError('target must be an object')
            if (data.get('reselection_required') is True
                    or data.get('reason') in ('SENSOR_DISCONNECTED', 'RESELECTION_REQUIRED')):
                self._revoke(data.get('reason') if data.get('reason') == 'SENSOR_DISCONNECTED'
                             else 'RESELECTION_REQUIRED')
                return
            if (data.get('valid') is not True or data.get('locked') is not True
                    or data.get('tracking_state') != 'OK'
                    or data.get('reason') != 'TRACKING_OK'):
                self._invalidate('TARGET_INVALID')
                return
            if (type(data.get('person_id')) is not int or data['person_id'] < 0
                    or type(data.get('observation_seq')) is not int
                    or data['observation_seq'] < 1
                    or data.get('reselection_required') is not False
                    or data.get('frame_id') != FRAME_ID):
                raise ValueError('invalid identity, sequence or frame')
            fields = ('timestamp', 'published_timestamp', 'age_sec',
                      'x_m', 'y_m', 'z_m', 'distance_m')
            if not all(number(data.get(k)) for k in fields):
                raise ValueError('missing or invalid numeric field')
            if (data['z_m'] <= 0 or data['distance_m'] <= 0
                    or abs(hypot(data['x_m'], data['z_m']) - data['distance_m']) > 0.02):
                raise ValueError('invalid camera position')
            if (data['timestamp'] <= 0 or not 0 <= data['age_sec'] <= self.timeout
                    or not -0.1 <= wall - data['timestamp'] <= self.timeout
                    or not -0.1 <= wall - data['published_timestamp'] <= self.timeout):
                self._invalidate('STALE_TARGET')
                return
            if self.authorized_id is not None and data['person_id'] != self.authorized_id:
                self._invalidate('ID_MISMATCH')
                return
            seq, stamp = data['observation_seq'], data['timestamp']
            if (self.sequence is None and self.capture_stamp is not None
                    and stamp <= self.capture_stamp):
                self._invalidate('STALE_TARGET')
                return
            if self.sequence is not None:
                if seq < self.sequence:
                    self._revoke('VISION_SEQUENCE_RESET')
                    return
                if seq == self.sequence and stamp == self.capture_stamp:
                    return  # Timer republication: never refresh freshness or validity.
                if seq <= self.sequence or stamp <= self.capture_stamp:
                    self._invalidate('STALE_TARGET')
                    return
            self.sequence, self.capture_stamp = seq, stamp
            self.target = data
            self.target_deadline = mono + self.timeout - max(0, wall - stamp, data['age_sec'])
            self.reason = 'TRACKING_OK'
            self._authorize()
        except (ValueError, TypeError):
            self._invalidate('INVALID_TARGET_MESSAGE')

    def guidance(self, *, wall, mono):
        self._watchdog(wall=wall, mono=mono)
        valid = self.target is not None
        distance = self.target['distance_m'] if valid else None
        result = {
            'state': self.mission.state.value, 'person_id': self.authorized_id,
            'distance_m': distance, 'desired_m': self.desired_m,
            'distance_error_m': distance - self.desired_m if valid else None,
            'tracking_valid': valid, 'linear_mps': 0.0, 'angular_rps': 0.0,
            'reason': self.reason, 'source_frame': FRAME_ID,
            'calibration_status': 'CALIBRATION_REQUIRED', 'motors_enabled': False,
            'reset_required': self.reset_required,
        }
        if self.reset_required:
            result['reason'] = 'RESET_REQUIRED'
        elif self.mission.state != State.FOLLOW:
            result['reason'] = 'MISSION_NOT_FOLLOW'
        elif self.authorized_id is None:
            result['reason'] = 'COME_AUTHORIZATION_REQUIRED'
        elif valid and self.target['person_id'] == self.authorized_id:
            # Approximate diagnostic mapping only, NOT a calibrated TF transform.
            twist = follow(self.target['z_m'], -self.target['x_m'],
                           desired_m=self.desired_m, tolerance_m=self.tolerance_m,
                           max_v=0.20, max_w=0.30)
            result.update(linear_mps=twist.linear, angular_rps=twist.angular)
            if distance < self.desired_m - self.tolerance_m:
                result['reason'] = 'TOO_CLOSE'
            elif distance <= self.desired_m + self.tolerance_m:
                result['reason'] = 'DISTANCE_DEADBAND'
            elif twist.linear == 0:
                result['reason'] = 'ROTATE_FIRST'
            else:
                result['reason'] = 'FOLLOWING'
        return result


def main(args=None):
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_msgs.msg import String

    class MissionControlNode(Node):
        def __init__(self):
            super().__init__('mission_control')
            self.controller = FollowController()
            self.mission = self.controller.mission
            self.state_pub = self.create_publisher(String, '/rover/state', 10)
            self.guidance_pub = self.create_publisher(String, '/rover/follow/guidance', 1)
            self.create_subscription(String, '/dev/cmd', self.on_command, 10)
            self.create_subscription(String, '/rover/gesture/event', self.on_gesture, 10)
            self.create_subscription(String, '/rover/vision/target', self.on_target, 1)
            self.create_timer(1.0, self.publish_state)
            self.watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.create_timer(0.05, self.publish_guidance, clock=self.watchdog_clock)
            self.get_logger().warn('DRY RUN: String guidance only; CALIBRATION_REQUIRED; '
                                   'motors disabled. Stage with /dev/cmd: start, arrived.')
            self.publish_state()

        @staticmethod
        def clocks():
            return {'wall': time.time(), 'mono': time.monotonic()}

        def on_command(self, msg):
            accepted = self.controller.command(msg.data, **self.clocks())
            self.get_logger().info(f'event={msg.data!r} accepted={accepted}')
            self.publish_guidance()
            self.publish_state()

        def on_gesture(self, msg):
            self.controller.receive_gesture(msg.data, **self.clocks())
            self.publish_guidance()
            self.publish_state()

        def on_target(self, msg):
            self.controller.receive_target(msg.data, **self.clocks())
            self.publish_guidance()
            self.publish_state()

        def publish_guidance(self):
            previous = self.mission.state
            data = self.controller.guidance(**self.clocks())
            self.guidance_pub.publish(String(data=json.dumps(data, allow_nan=False)))
            if self.mission.state != previous:
                self.publish_state()

        def publish_state(self):
            msg = String()
            msg.data = self.mission.state.value
            self.state_pub.publish(msg)

    rclpy.init(args=args)
    node = None
    try:
        node = MissionControlNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
