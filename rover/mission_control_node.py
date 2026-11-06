"""Mission destination intake plus motor-disabled, 2 m FOLLOW diagnostics.

FOLLOW authorizes the WAIT_COMMAND -> follow transition for one vision
ID. CAMERA/IMAGE Z -> forward, -X -> left is approximate: calibration required.
Only String state/guidance publishers exist; this is not a motor controller.
"""
import json
from math import hypot
import time

from rover.mission import Mission, State
from rover.nav import GNSSGoal, follow
from rover.vision import FRAME_ID, number, strict_json


class FollowController:
    """Testable policy around the existing Mission; no ROS or hardware access.

    Supplied monotonic times enforce both message and capture-age watchdogs.
    Equal cached snapshots never renew an observation's deadline or recover it.
    Tracking loss revokes authorization until a new FOLLOW gesture and lock.
    Explicit STOP/STAY remain latched against gestures.
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
        self.awaiting_follow = False
        self.destination = None
        self.destination_stamp = None

    def request_destination(self, text, *, wall):
        """Accept an operator destination without pretending navigation exists.

        This endpoint is separate from /dev/cmd's simulated start/arrived events.
        A future navigation adapter must supply dispatch, cancellation, sensor
        freshness and verified arrival before this can transition to GO/WAIT.
        """
        reply = dict(request_id=None, destination_accepted=False,
                     navigation_started=False, state=self.mission.state.value,
                     reason='INVALID_DESTINATION')
        try:
            data = strict_json(text)
            if not isinstance(data, dict):
                return reply
            request_id = data.get('request_id')
            if not isinstance(request_id, str) or not 1 <= len(request_id) <= 64:
                return reply
            reply['request_id'] = request_id
            if set(data) != {'request_id', 'timestamp', 'latitude', 'longitude'}:
                return reply
            goal = GNSSGoal(data['latitude'], data['longitude'])
            stamp = data['timestamp']
            if (not number(stamp) or stamp <= 0 or not -0.1 <= wall - stamp <= 5.0
                    or (self.destination_stamp is not None and stamp <= self.destination_stamp)):
                reply['reason'] = 'STALE_OR_REPLAYED_REQUEST'
                return reply
        except (ValueError, TypeError):
            return reply
        if self.mission.state != State.READY:
            reply['reason'] = 'MISSION_NOT_READY'
            return reply
        self.destination = goal
        self.destination_stamp = stamp
        reply.update(destination_accepted=True, reason='NAVIGATION_UNAVAILABLE',
                     latitude=goal.latitude, longitude=goal.longitude)
        return reply

    def _invalidate(self, reason, *, wall=None):
        self.target = None
        self.target_deadline = None
        self.reason = reason
        if self.authorized_id is not None:
            self.authorized_id = None
            self.pending = None
            self.mission.event('tracking_lost', source='tracking_watchdog')
            self.awaiting_follow = True
            self.wait_since = wall
            # A new camera session may restart its sequence, but never its
            # capture-time watermark. Cached frames cannot renew authorization.
            self.sequence = None

    def _watchdog(self, *, wall, mono):
        if self.last_receive is not None and mono - self.last_receive > self.timeout:
            self._invalidate('SENSOR_DISCONNECTED', wall=wall)
        elif (self.target is not None
              and (mono > self.target_deadline
                   or wall - self.target['timestamp'] > self.timeout
                   or self.target['timestamp'] - wall > 0.1)):
            self._invalidate('STALE_TARGET', wall=wall)
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
        self.awaiting_follow = False
        if self.mission.state in (State.STOP, State.READY):
            self.destination = None
        if self.mission.state != State.FOLLOW:
            self.authorized_id = None
        if self.mission.state == State.WAIT:
            self.wait_since = wall
        if previous == State.STOP and self.mission.state == State.READY:
            self.reset_required = False
            self._invalidate('NO_TARGET')
            self.last_receive = self.sequence = None
            # A restarted vision process may reset sequence, but old captures and
            # FOLLOW events must remain ineligible even across an operator reset.
        return True

    def receive_gesture(self, text, *, wall, mono):
        self._watchdog(wall=wall, mono=mono)
        try:
            event = strict_json(text)
            if (not isinstance(event, dict) or event.get('gesture') != 'FOLLOW'
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
                or (self.awaiting_follow and stamp <= self.wait_since)
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
            self.awaiting_follow = False
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
                self._invalidate(data.get('reason') if data.get('reason') == 'SENSOR_DISCONNECTED'
                                 else 'RESELECTION_REQUIRED', wall=wall)
                return
            if (data.get('valid') is not True or data.get('locked') is not True
                    or data.get('tracking_state') != 'OK'
                    or data.get('reason') != 'TRACKING_OK'):
                self._invalidate('TARGET_INVALID', wall=wall)
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
                self._invalidate('STALE_TARGET', wall=wall)
                return
            if self.authorized_id is not None and data['person_id'] != self.authorized_id:
                self._invalidate('ID_MISMATCH', wall=wall)
                return
            seq, stamp = data['observation_seq'], data['timestamp']
            if (self.sequence is None and self.capture_stamp is not None
                    and stamp <= self.capture_stamp):
                self._invalidate('STALE_TARGET', wall=wall)
                return
            if self.sequence is not None:
                if seq < self.sequence:
                    self._invalidate('VISION_SEQUENCE_RESET', wall=wall)
                    return
                if seq == self.sequence and stamp == self.capture_stamp:
                    return  # Timer republication: never refresh freshness or validity.
                if seq <= self.sequence or stamp <= self.capture_stamp:
                    self._invalidate('STALE_TARGET', wall=wall)
                    return
            self.sequence, self.capture_stamp = seq, stamp
            self.target = data
            self.target_deadline = mono + self.timeout - max(0, wall - stamp, data['age_sec'])
            self.reason = 'TRACKING_OK'
            self._authorize()
        except (ValueError, TypeError):
            self._invalidate('INVALID_TARGET_MESSAGE', wall=wall)

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
            'published_timestamp': wall,
            'follow_gesture_allowed': self.mission.state == State.WAIT and not self.reset_required,
            'tracking_reason': self.reason,
        }
        if self.reset_required:
            result['reason'] = 'RESET_REQUIRED'
        elif self.awaiting_follow and self.mission.state == State.WAIT:
            result['reason'] = 'FOLLOW_GESTURE_REQUIRED'
        elif self.mission.state != State.FOLLOW:
            result['reason'] = 'MISSION_NOT_FOLLOW'
        elif self.authorized_id is None:
            result['reason'] = 'FOLLOW_AUTHORIZATION_REQUIRED'
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


def prompt_destination(read=input, write=print):
    """Ask for decimal degrees; invalid input never creates a destination."""
    values = []
    for name, limit in (('latitude', 90), ('longitude', 180)):
        while True:
            raw = read(f'Destination {name} (decimal degrees, {-limit} to {limit}): ')
            try:
                value = float(raw)
                if not number(value) or not -limit <= value <= limit:
                    raise ValueError
            except ValueError:
                write(f'Enter a finite number between {-limit} and {limit}.')
                continue
            values.append(value)
            break
    return GNSSGoal(*values)


def start_mission(args=None):
    """Interactive ROS destination client; reports blocked navigation honestly."""
    from uuid import uuid4

    try:
        goal = prompt_destination()
    except (EOFError, KeyboardInterrupt):
        print('\nCancelled; no destination sent.')
        raise SystemExit(130)
    print(f'Destination: latitude={goal.latitude}, longitude={goal.longitude}')

    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
        from std_msgs.msg import String
    except ImportError:
        print('ROS 2 is unavailable. Source ROS and the built workspace, then try again.')
        raise SystemExit(1)

    rclpy.init(args=args)
    node = None
    exit_code = 1
    try:
        request_id = uuid4().hex
        node = rclpy.create_node('start_mission_' + request_id[:8])
        replies = []

        def receive(msg):
            try:
                data = strict_json(msg.data)
                if (isinstance(data, dict) and data.get('request_id') == request_id
                        and type(data.get('destination_accepted')) is bool
                        and type(data.get('navigation_started')) is bool
                        and isinstance(data.get('reason'), str)):
                    replies.append(data)
            except ValueError:
                pass

        node.create_subscription(String, '/rover/mission/start_status', receive, 10)
        publisher = node.create_publisher(String, '/rover/mission/start_request', 10)
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and time.monotonic() < deadline:
            if (publisher.get_subscription_count() == 1
                    and node.count_publishers('/rover/mission/start_status') == 1):
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        if (publisher.get_subscription_count() != 1
                or node.count_publishers('/rover/mission/start_status') != 1):
            print('Exactly one mission controller is required. Start it with:')
            print('  ros2 launch mars_rover mars_rover.launch.py')
        else:
            publisher.publish(String(data=json.dumps(dict(
                request_id=request_id, timestamp=time.time(),
                latitude=goal.latitude, longitude=goal.longitude), allow_nan=False)))
            deadline = time.monotonic() + 5.0
            while rclpy.ok() and not replies and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            if not replies:
                print('No acknowledgement received. Destination acceptance is unconfirmed.')
            else:
                reply = replies[0]
                if reply['destination_accepted']:
                    print('Mission controller received your destination.')
                if reply['navigation_started'] and reply['destination_accepted']:
                    print('Navigation started.')
                    exit_code = 0
                elif reply['reason'] == 'NAVIGATION_UNAVAILABLE':
                    print('Rover remains stopped: the drive/navigation interface is not connected yet.')
                    print('The destination is held in this controller session; no movement was requested.')
                    exit_code = 2
                else:
                    print('Navigation did not start: ' + reply['reason'])
    except (KeyboardInterrupt, ExternalShutdownException):
        print('\nClient interrupted. Check mission status before retrying.')
        exit_code = 130
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


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
            self.destination_pub = self.create_publisher(String, '/rover/mission/start_status', 10)
            self.create_subscription(String, '/rover/mission/start_request', self.on_start_request, 10)
            self.create_subscription(String, '/dev/cmd', self.on_command, 10)
            self.create_subscription(String, '/rover/gesture/event', self.on_gesture, 10)
            self.create_subscription(String, '/rover/vision/target', self.on_target, 1)
            self.create_timer(1.0, self.publish_state)
            self.watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.create_timer(0.05, self.publish_guidance, clock=self.watchdog_clock)
            self.get_logger().warn('Navigation unavailable; motors disabled. Enter a destination with '
                                   'ros2 run mars_rover start_mission. /dev/cmd events are simulation only.')
            self.publish_state()

        def on_start_request(self, msg):
            reply = self.controller.request_destination(msg.data, wall=time.time())
            self.destination_pub.publish(String(data=json.dumps(reply, allow_nan=False)))
            self.get_logger().info(f"Destination request: {reply['reason']}; "
                                   'navigation_started=false')
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
