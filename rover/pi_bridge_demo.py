"""Display Jetson FOLLOW intentions only; no Pi connection or actuator output.

Current guidance has no producer timestamp: receipt timeout is enforceable, but
source age/replay cannot be established. Every physical action remains blocked.
An optional timestamp, if supplied, is checked without requiring upstream changes.
"""
import json
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from rover.mission import State
from rover.vision import FRAME_ID, number, strict_json


class GuidanceMonitor:
    """Small validation/status core. This never returns permission to actuate."""
    timeout = 0.5

    def __init__(self):
        self.data = None
        self.received = False
        self.received_at = None
        self.source_watermark = None
        self.source_verified = False
        self.deadline = None
        self.reason = 'NO_GUIDANCE'

    def receive(self, text, *, wall, mono):
        self.received = True
        self.data = None
        self.source_verified = False
        self.reason = 'INVALID_GUIDANCE'
        try:
            data = strict_json(text)
            required = {'state', 'person_id', 'tracking_valid', 'reset_required', 'motors_enabled',
                        'source_frame', 'calibration_status', 'reason', 'desired_m',
                        'distance_m', 'distance_error_m', 'linear_mps', 'angular_rps'}
            if not isinstance(data, dict) or not required <= data.keys():
                return
            if (data.get('state') not in {state.value for state in State}
                    or type(data.get('tracking_valid')) is not bool
                    or type(data.get('reset_required')) is not bool
                    or data.get('motors_enabled') is not False
                    or data.get('source_frame') != FRAME_ID
                    or data.get('calibration_status') not in ('CALIBRATION_REQUIRED', 'CALIBRATED')
                    or not isinstance(data.get('reason'), str) or not data['reason']):
                return
            pid = data.get('person_id')
            if pid is not None and (type(pid) is not int or pid < 0):
                return
            if not all(number(data.get(key)) for key in ('desired_m', 'linear_mps', 'angular_rps')):
                return
            if (data['desired_m'] <= 0 or not 0 <= data['linear_mps'] <= 0.20
                    or abs(data['angular_rps']) > 0.30):
                return
            if data['tracking_valid']:
                if (not number(data.get('distance_m')) or data['distance_m'] <= 0
                        or not number(data.get('distance_error_m'))
                        or abs(data['distance_error_m'] - (data['distance_m'] - data['desired_m'])) > 1e-6):
                    return
            elif data.get('distance_m') is not None or data.get('distance_error_m') is not None:
                return
            if data['linear_mps'] != 0 or data['angular_rps'] != 0:
                if (data['state'] != 'FOLLOW' or not data['tracking_valid'] or pid is None
                        or data['reset_required']):
                    return
            if 'timestamp' in data:
                stamp = data['timestamp']
                if (not number(stamp) or stamp <= 0 or not -0.1 <= wall - stamp <= self.timeout
                        or (self.source_watermark is not None and stamp <= self.source_watermark)):
                    self.reason = 'STALE_OR_REPLAYED_GUIDANCE'
                    return
                self.source_watermark = stamp
                self.source_verified = True
            self.data = data
            self.received_at = mono
            source_age = max(0., wall - data['timestamp']) if self.source_verified else 0.
            self.deadline = mono + self.timeout - source_age
            self.reason = 'DIAGNOSTIC_ONLY'
        except (ValueError, TypeError):
            return

    def snapshot(self, *, wall, mono):
        data = self.data
        reason = self.reason
        if data is not None:
            age = wall - data['timestamp'] if self.source_verified else 0.
            if mono > self.deadline or not -0.1 <= age <= self.timeout:
                data = None
                reason = 'STALE_GUIDANCE'
        blocked = ['PI_NOT_CONNECTED', 'MOTORS_DISABLED', 'NO_ACTUATOR_INTERFACE']
        if not data:
            blocked.insert(0, reason)
        else:
            if data['calibration_status'] != 'CALIBRATED':
                blocked.insert(0, 'CALIBRATION_REQUIRED')
            if not self.source_verified:
                blocked.insert(0, 'SOURCE_TIME_UNAVAILABLE')
            if data['state'] != 'FOLLOW' or not data['tracking_valid'] or data['reset_required']:
                blocked.insert(0, 'FOLLOW_NOT_READY')
        return dict(mode='DRY_RUN', guidance_received=self.received, guidance_valid=data is not None,
                    state=data['state'] if data else None,
                    person_id=data['person_id'] if data else None,
                    requested_linear_mps=data['linear_mps'] if data else 0.,
                    requested_angular_rps=data['angular_rps'] if data else 0.,
                    source_freshness_verified=bool(data and self.source_verified),
                    receipt_age_sec=max(0., mono - self.received_at) if self.received_at is not None else None,
                    reason=reason, blocked_reasons=blocked, action_allowed=False,
                    pi_connected=False, motors_enabled=False, command_executed=False)


class PiBridgeDemo(Node):
    def __init__(self, **kwargs):
        super().__init__('pi_bridge_demo', **kwargs)
        self.monitor = GuidanceMonitor()
        self.pub = self.create_publisher(String, '/rover/demo/pi_bridge/status', 1)
        self.create_subscription(String, '/rover/follow/guidance', self.on_guidance, 1)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.1, self.publish_status, clock=self.steady_clock)
        self.get_logger().info('DEMO ONLY: no Pi transport; no physical actions are allowed')

    def on_guidance(self, msg):
        self.monitor.receive(msg.data, wall=time.time(), mono=time.monotonic())
        self.publish_status()

    def publish_status(self):
        status = self.monitor.snapshot(wall=time.time(), mono=time.monotonic())
        self.pub.publish(String(data=json.dumps(status, allow_nan=False)))
        self.get_logger().info(
            f"DRY_RUN {status['reason']}: forward={status['requested_linear_mps']:.3f} m/s, "
            f"turn={status['requested_angular_rps']:.3f} rad/s; action_allowed=false",
            throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PiBridgeDemo()
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
