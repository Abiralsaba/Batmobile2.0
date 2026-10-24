"""Detect COME from any person and request vision target lock.

Perception only: never sends motor commands.
"""

import json
import math
import time
from collections import deque
from statistics import median

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from rover.vision import strict_json, validate_message, VisionConfig, number


ARMS = {
    "RIGHT": (2, 3, 4),
    "LEFT": (5, 6, 7)
}


def valid_point(p):
    return (
        isinstance(p, list)
        and len(p) == 3
        and all(
            type(x) in (int, float) and math.isfinite(x)
            for x in p
        )
    )


def parse_json(text):
    return strict_json(text)


class ComeMotion:
    """Recognize DOWN -> UP -> DOWN wrist movement."""

    def __init__(self):
        self.samples = deque(maxlen=3)
        self.phase = 0
        self.up_at = None
        self.last_at = None

    def update(self, dy, now):
        if not number(dy) or not number(now):
            self.__init__()
            return False
        if self.last_at is not None:
            if now <= self.last_at:
                self.__init__()
                return False
            if now - self.last_at > 0.7:
                self.__init__()
        self.last_at = now
        self.samples.append(dy)

        if len(self.samples) < 3:
            return False

        y = median(self.samples)

        if self.phase == 0 and y > 0.12:
            self.phase = 1

        elif self.phase == 1 and y < -0.09:
            self.phase = 2
            self.up_at = now

        elif self.phase == 2:
            if now - self.up_at > 3.5:
                self.__init__()

            elif y > 0.12 and now - self.up_at >= 0.25:
                self.__init__()
                return True

        return False


class GestureNode(Node):

    def __init__(self):
        super().__init__("gesture")

        self.detectors = {}
        self.last_seen = {}
        self.last_stamp = None
        self.last_raw_at = None

        self.pending = None
        self.pending_arm = None
        self.request_wall = None
        self.request_mono = None

        self.finished = False
        self.validation_config = VisionConfig()

        raw_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.create_subscription(
            String,
            "/rover/vision/bodies",
            self.on_bodies,
            raw_qos
        )

        self.create_subscription(
            String,
            "/rover/vision/target",
            self.on_target,
            10
        )

        self.select_pub = self.create_publisher(
            String, "/rover/vision/select", 10
        )

        self.event_pub = self.create_publisher(
            String, "/rover/gesture/event", 10
        )

        self.watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.2, self.watchdog, clock=self.watchdog_clock)

        self.get_logger().info(
            "COME detection ready: ALL PEOPLE, motors disabled"
        )

    def clear_attempt(self):
        self.detectors.clear()
        self.last_seen.clear()
        self.pending = None
        self.pending_arm = None
        self.request_wall = None
        self.request_mono = None

    def watchdog(self):
        now = time.monotonic()

        if (
            self.last_raw_at is not None
            and now - self.last_raw_at > 0.7
        ):
            self.clear_attempt()
            self.last_raw_at = None
            # Keep the timestamp watermark: a disconnect cannot legitimize replay.

        if (
            self.pending is not None
            and now - self.request_mono > 3.0
        ):
            self.get_logger().warning(
                "Vision did not confirm selection"
            )
            self.clear_attempt()

    def on_bodies(self, msg):
        if self.finished:
            return

        self.watchdog()

        try:
            data = validate_message(msg.data, self.validation_config)

            stamp = data["timestamp"]
            age = time.time() - stamp

            if (
                type(stamp) not in (int, float)
                or not math.isfinite(stamp)
                or age < -0.2
                or age > 0.5
            ):
                self.clear_attempt()
                return

        except (ValueError, TypeError, KeyError):
            self.clear_attempt()
            return

        if (
            self.last_stamp is not None
            and stamp <= self.last_stamp
        ):
            self.clear_attempt()
            return

        if self.last_stamp is not None and stamp - self.last_stamp > 0.7:
            self.clear_attempt()
        self.last_stamp = stamp
        now = time.monotonic()
        self.last_raw_at = now

        if self.pending is not None:
            # Continue checking the gesturing person while vision acquires it.
            person = next((p for p in data['people'] if p['id'] == self.pending), None)
            if (person is None or person['tracking_state'] != 'OK'
                    or not all(valid_point(person['keypoints_m'][i])
                               for i in ARMS[self.pending_arm])):
                self.clear_attempt()
            return

        events = []
        seen = set()

        for person in data["people"]:
            if (
                not isinstance(person, dict)
                or person.get("tracking_state") != "OK"
            ):
                continue

            pid = person.get("id")
            joints = person.get("keypoints_m")

            if (
                type(pid) is not int
                or pid < 0
                or not isinstance(joints, list)
                or len(joints) != 18
            ):
                continue

            seen.add(pid)

            if (
                pid in self.last_seen
                and now - self.last_seen[pid] > 0.7
            ):
                self.detectors.pop(pid, None)

            self.last_seen[pid] = now

            if pid not in self.detectors:
                self.detectors[pid] = {
                    side: ComeMotion() for side in ARMS
                }

            for side, (s, e, w) in ARMS.items():
                shoulder = joints[s]
                elbow = joints[e]
                wrist = joints[w]

                if not all(
                    valid_point(p)
                    for p in (shoulder, elbow, wrist)
                ):
                    self.detectors[pid][side] = ComeMotion()
                    continue

                dy = wrist[1] - elbow[1]

                # Gesture durations use capture time, not callback scheduling.
                if self.detectors[pid][side].update(dy, stamp):
                    events.append((pid, side))

        for pid in set(self.detectors) - seen:
            self.detectors.pop(pid, None)
            self.last_seen.pop(pid, None)

        # Reject simultaneous gestures from different people.
        if len(set(pid for pid, _ in events)) != 1:
            return

        if events:
            pid, side = events[0]

            self.pending = pid
            self.pending_arm = side
            self.request_wall = time.time()
            self.request_mono = now

            command = String()
            command.data = json.dumps({
                "action": "select",
                "person_id": pid
            }, allow_nan=False)

            self.select_pub.publish(command)

            self.get_logger().info(
                f"COME FROM ID={pid} ARM={side}; requesting lock"
            )

    def on_target(self, msg):
        self.watchdog()
        if self.finished or self.pending is None:
            return

        try:
            data = parse_json(msg.data)

            if not isinstance(data, dict):
                return

            age = data.get("age_sec")
            stamp = data.get("timestamp")

            fresh = (
                number(age)
                and 0 <= age < 0.4
                and number(stamp)
                and stamp > self.request_wall
                and 0 <= time.time() - stamp < 0.4
                and self.last_raw_at is not None
                and time.monotonic() - self.last_raw_at <= 0.7
            )

            confirmed = (
                data.get("valid") is True
                and data.get("locked") is True
                and data.get("tracking_state") == "OK"
                and type(data.get("person_id")) is int
                and data.get("person_id") == self.pending
                and fresh
            )

            if not confirmed:
                return

        except (ValueError, TypeError):
            return

        event = String()
        event.data = json.dumps({
            "gesture": "COME",
            "confirmed": True,
            "person_id": self.pending,
            "arm": self.pending_arm,
            "timestamp": time.time()
        }, allow_nan=False)

        self.event_pub.publish(event)

        self.get_logger().info(
            f"COME CONFIRMED! ID={self.pending} LOCKED"
        )

        # Only one COME activation per run.
        self.finished = True


def main():
    rclpy.init()
    node = GestureNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
