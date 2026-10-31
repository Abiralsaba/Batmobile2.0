"""ROS 2 EVA backend: hardware isolation, state aggregation and command gate."""
from collections import deque
import argparse
import json
import math
import os
import platform
import queue
import sys
import time
import uuid

import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String

from eva_suit.sensors import FlexGestureLatch, FlexReader, GNSSReader
from eva_suit.voice import VoiceWorker


def load_config(path=""):
    path = path or os.path.join(get_package_share_directory("eva_suit"), "config", "eva.yaml")
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    for section in ("serial", "voice", "commands", "ros"):
        if not isinstance(data.get(section), dict):
            raise ValueError(f"missing configuration section: {section}")
    return data, path


class CommandGate:
    """Transport-neutral safety checks shared by voice and touchscreen requests."""
    def __init__(self, allowed, cooldown_sec):
        self.allowed = {str(item).upper() for item in allowed}
        self.cooldown = float(cooldown_sec)
        self.last = {}

    def validate(self, command, source, *, rover_online, now=None):
        now = time.monotonic() if now is None else now
        command, source = str(command or "").strip().upper(), str(source or "").strip().upper()
        if source not in ("VOICE", "TOUCH", "FLEX"):
            return False, "INVALID_SOURCE", command, source
        if command not in self.allowed:
            return False, "UNSUPPORTED_COMMAND", command, source
        if not rover_online:
            return False, "ROVER_DISCONNECTED", command, source
        if now - self.last.get((source, command), -math.inf) < self.cooldown:
            return False, "DUPLICATE_COOLDOWN", command, source
        self.last[(source, command)] = now
        return True, "VALIDATED", command, source


class EVABackend(Node):
    def __init__(self, config, config_path):
        super().__init__("eva_backend")
        self.config, self.config_path = config, config_path
        self.inbox = queue.SimpleQueue()
        self.events = deque(maxlen=40)
        self.workers = []
        self.pending = {}
        self.counter = 0
        self.boot_id = uuid.uuid4().hex[:8]
        self.last_rover = None
        self.last_rover_state = "UNKNOWN"
        self.last_errors = deque(maxlen=5)
        self.ack_available = False
        self.gate = CommandGate(config["commands"].get("allowed", []),
                                config["commands"].get("cooldown_sec", 1.5))
        self.flex_latch = FlexGestureLatch()
        self.ack_timeout = float(config["commands"].get("ack_timeout_sec", 3.0))
        self.rover_timeout = float(config["commands"].get("rover_timeout_sec", 3.0))
        if min(self.ack_timeout, self.rover_timeout) <= 0:
            raise ValueError("command timeouts must be positive")

        transient = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                               reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.gnss_pub = self.create_publisher(NavSatFix, "/eva/gnss/fix", 10)
        self.gnss_status_pub = self.create_publisher(String, "/eva/gnss/status", transient)
        self.flex_pub = self.create_publisher(String, "/eva/flex/raw", 10)
        self.flex_status_pub = self.create_publisher(String, "/eva/flex/status", transient)
        self.voice_pub = self.create_publisher(String, "/eva/voice/event", 10)
        self.voice_status_pub = self.create_publisher(String, "/eva/voice/status", transient)
        self.command_pub = self.create_publisher(String, "/eva/command/request", 10)
        self.command_status_pub = self.create_publisher(String, "/eva/command/status", transient)
        self.system_pub = self.create_publisher(String, "/eva/system/status", transient)

        self.create_subscription(String, "/eva/command/input", self._on_command_input, 10)
        self.create_subscription(String, "/eva/command/ack", self._on_ack, 10)
        self.create_subscription(String, "/rover/state", self._on_rover_state, 10)
        self.create_subscription(String, "/rover/gesture/event", self._on_rover_activity, 10)
        self.create_subscription(String, "/rover/vision/target", self._on_rover_activity, 1)
        self.create_subscription(String, "/rover/follow/guidance", self._on_rover_activity, 1)
        self.create_timer(0.05, self._drain_inbox)
        self.create_timer(0.25, self._watchdog)

        serial_cfg = config["serial"]
        gnss_cfg, flex_cfg = serial_cfg["gnss"], serial_cfg["flex"]
        gnss = GNSSReader(gnss_cfg, serial_cfg,
                          lambda data: self.inbox.put(("gnss_data", data)),
                          lambda data: self.inbox.put(("gnss_status", data)),
                          excluded=(flex_cfg.get("port", ""),))
        flex = FlexReader(flex_cfg, serial_cfg,
                          lambda data: self.inbox.put(("flex_data", data)),
                          lambda data: self.inbox.put(("flex_status", data)),
                          excluded=(gnss_cfg.get("port", ""),))
        voice = VoiceWorker(config["voice"],
                            lambda data: self.inbox.put(("voice_event", data)),
                            lambda data: self.inbox.put(("voice_status", data)))
        self.workers.extend((gnss, flex, voice))
        for worker in self.workers:
            worker.start()
        self._publish_system()
        self.get_logger().warning(
            "EVA backend is non-actuating. Rover acknowledgements remain unavailable until "
            "a rover-side /eva/command/ack publisher is independently deployed.")

    @staticmethod
    def _string(data):
        msg = String()
        msg.data = json.dumps(data, separators=(",", ":"), allow_nan=False)
        return msg

    def _rover_online(self):
        return self.last_rover is not None and time.monotonic() - self.last_rover <= self.rover_timeout

    def _on_rover_activity(self, _msg):
        self.last_rover = time.monotonic()

    def _on_rover_state(self, msg):
        self.last_rover = time.monotonic()
        try:
            data = json.loads(msg.data)
            self.last_rover_state = str(data.get("state", msg.data)) if isinstance(data, dict) else msg.data
        except json.JSONDecodeError:
            self.last_rover_state = msg.data[:80]

    def _on_command_input(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                raise ValueError("command input must be an object")
            stamp = data.get("timestamp")
            if stamp is not None and (not isinstance(stamp, (int, float))
                                      or abs(time.time() - stamp) > 5.0):
                raise ValueError("stale or invalid input timestamp")
            self._request_command(data.get("command"), data.get("source", "TOUCH"))
        except (json.JSONDecodeError, ValueError) as exc:
            self._publish_command_status("", "", "REJECTED", str(exc))

    def _request_command(self, command, source):
        accepted, reason, command, source = self.gate.validate(
            command, source, rover_online=self._rover_online())
        if not accepted:
            self._publish_command_status("", command, "REJECTED", reason, source=source)
            return False, reason
        if command == "STOP":
            for old_id, old in list(self.pending.items()):
                self._publish_command_status(old_id, old["command"], "SUPERSEDED",
                                             "SUPERSEDED_BY_STOP", source=old["source"])
            self.pending.clear()
        self.counter += 1
        command_id = f"eva-{self.boot_id}-{self.counter:06d}"
        payload = {"command_id": command_id, "source": source, "command": command,
                   "timestamp": time.time(), "requires_ack": True}
        self.command_pub.publish(self._string(payload))
        now = time.monotonic()
        self.pending[command_id] = {**payload, "deadline": now + self.ack_timeout,
                                    "expiry": now + 300.0}
        self._publish_command_status(command_id, command, "PUBLISHED", "AWAITING_ROVER_ACK",
                                     source=source)
        return True, command_id

    def _on_ack(self, msg):
        try:
            ack = json.loads(msg.data)
            command_id = ack.get("command_id")
            pending = self.pending.get(command_id)
            if not isinstance(ack, dict) or pending is None:
                raise ValueError("unknown or expired command_id")
            if ack.get("command") != pending["command"]:
                raise ValueError("ack command mismatch")
            status = str(ack.get("status", "")).upper()
            if status not in ("RECEIVED", "ACCEPTED", "REJECTED", "COMPLETED", "FAILED"):
                raise ValueError("invalid acknowledgement status")
            stamp = ack.get("timestamp")
            if not isinstance(stamp, (int, float)) or abs(time.time() - stamp) > 10.0:
                raise ValueError("stale acknowledgement")
            self.ack_available = True
            self.last_rover = time.monotonic()
            self._publish_command_status(command_id, pending["command"], status,
                                         str(ack.get("reason", "")), source=pending["source"])
            if status in ("REJECTED", "COMPLETED", "FAILED"):
                self.pending.pop(command_id, None)
            elif status == "RECEIVED":
                pending["deadline"] = time.monotonic() + self.ack_timeout
            elif status == "ACCEPTED":
                pending["deadline"] = None
        except (json.JSONDecodeError, ValueError, AttributeError) as exc:
            self.last_errors.append(f"ACK: {exc}")

    def _publish_command_status(self, command_id, command, status, reason, *, source=""):
        payload = {"command_id": command_id, "command": command, "source": source,
                   "status": status, "reason": reason, "timestamp": time.time(),
                   "ack_available": self.ack_available}
        self.command_status_pub.publish(self._string(payload))
        self.events.append(payload)

    def _drain_inbox(self):
        for _ in range(100):
            try:
                kind, data = self.inbox.get_nowait()
            except queue.Empty:
                break
            try:
                if kind == "gnss_data":
                    self._publish_fix(data)
                elif kind == "gnss_status":
                    self.gnss_status_pub.publish(self._string(data))
                elif kind == "flex_data":
                    command = self.flex_latch.process(data)
                    dispatched = None
                    if command is not None:
                        accepted, result = self._request_command(command, "FLEX")
                        dispatched = {"command": command,
                                      "status": "PUBLISHED" if accepted else "REJECTED",
                                      "result": result}
                    payload = {**data, "received_timestamp": time.time(),
                               "format": data.get("format", "PROVISIONAL_JSON_V1")}
                    if dispatched is not None:
                        payload["command_dispatch"] = dispatched
                    self.flex_pub.publish(self._string(payload))
                elif kind == "flex_status":
                    if data.get("state") in ("DISCONNECTED", "STALE", "INVALID_DATA"):
                        command = self.flex_latch.fail_safe()
                        if command is not None:
                            self._request_command(command, "FLEX")
                    self.flex_status_pub.publish(self._string(data))
                elif kind == "voice_status":
                    self.voice_status_pub.publish(self._string(data))
                elif kind == "voice_event":
                    data["timestamp"] = time.time()
                    if data.get("accepted") and data.get("command") == "STATUS":
                        data["delivery"] = "LOCAL_STATUS_ONLY"
                    elif data.get("accepted"):
                        ok, result = self._request_command(data["command"], "VOICE")
                        data["delivery"] = "PUBLISHED" if ok else "REJECTED"
                        data["delivery_reason"] = result
                    self.voice_pub.publish(self._string(data))
            except (TypeError, ValueError) as exc:
                self.last_errors.append(f"{kind}: {exc}")

    def _publish_fix(self, data):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.config["serial"]["gnss"].get("frame_id", "eva_gnss_link")
        msg.status.status = NavSatStatus.STATUS_FIX if data["valid"] else NavSatStatus.STATUS_NO_FIX
        msg.status.service = (NavSatStatus.SERVICE_GPS
                              if data["valid"] and data.get("talker") == "GP" else 0)
        msg.latitude, msg.longitude, msg.altitude = (
            data["latitude"], data["longitude"], data["altitude"])
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.gnss_pub.publish(msg)

    def _watchdog(self):
        now = time.monotonic()
        for command_id, item in list(self.pending.items()):
            if now > item["expiry"]:
                self.pending.pop(command_id, None)
            elif item["deadline"] is not None and now > item["deadline"]:
                self.pending.pop(command_id, None)
                self._publish_command_status(command_id, item["command"], "ACK_TIMEOUT",
                                             "ROVER_ACK_UNAVAILABLE", source=item["source"])
        self._publish_system()

    def _publish_system(self):
        memory_available = None
        try:
            with open("/proc/meminfo", "r", encoding="utf-8") as stream:
                fields = {line.split(":", 1)[0]: line.split()[1] for line in stream if ":" in line}
            memory_available = round(int(fields["MemAvailable"]) / 1024, 1)
        except (FileNotFoundError, KeyError, ValueError):
            pass
        temperature = None
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r", encoding="ascii") as stream:
                temperature = round(float(stream.read().strip()) / 1000, 1)
        except (FileNotFoundError, ValueError):
            pass
        payload = {"backend": "HEALTHY", "rover_connected": self._rover_online(),
                   "rover_state": self.last_rover_state, "ack_available": self.ack_available,
                   "pending_commands": len(self.pending), "config_file": self.config_path,
                   "last_errors": list(self.last_errors), "timestamp": time.time(),
                   "simulation": False, "host": {"architecture": platform.machine(),
                   "os": platform.platform(), "load_1m": round(os.getloadavg()[0], 2),
                   "memory_available_mib": memory_available, "temperature_c": temperature}}
        self.system_pub.publish(self._string(payload))

    def destroy_node(self):
        for worker in self.workers:
            worker.stop()
        return super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser(description="Batmobile EVA suit ROS backend")
    parser.add_argument("--config", default="", help="EVA YAML configuration path")
    source_args = sys.argv[1:] if args is None else list(args)
    known, ros_args = parser.parse_known_args(source_args)
    config, path = load_config(known.config)
    domain = config["ros"].get("domain_id")
    if domain is not None:
        domain = int(domain)
        if not 0 <= domain <= 232:
            raise ValueError("ROS domain_id must be between 0 and 232")
        os.environ["ROS_DOMAIN_ID"] = str(domain)
    os.environ["ROS_LOCALHOST_ONLY"] = "1" if config["ros"].get("localhost_only") else "0"
    rclpy.init(args=ros_args)
    node = None
    try:
        node = EVABackend(config, path)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
