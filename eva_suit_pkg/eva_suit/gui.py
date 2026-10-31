"""Compact PyQt5 EVA mission terminal; ROS subscriptions only, no hardware access."""
from collections import deque
import argparse
import json
import math
import os
import sys
import time

import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from PyQt5.QtCore import QDateTime, Qt, QTimer
from PyQt5.QtGui import QColor, QPainter
from PyQt5.QtNetwork import QLocalServer, QLocalSocket
from PyQt5.QtWidgets import (
    QApplication, QFrame, QGridLayout, QHBoxLayout, QLabel, QMainWindow,
    QPushButton, QStackedWidget, QVBoxLayout, QWidget,
)


GREEN, CYAN, AMBER, RED, GRAY = "#69ff94", "#45d9ff", "#ffbe55", "#ff5c6c", "#687481"


def decode(text):
    try:
        data = json.loads(text)
        return data if isinstance(data, dict) else {"value": text}
    except json.JSONDecodeError:
        return {"value": text}


class Dot(QWidget):
    def __init__(self, color=GRAY):
        super().__init__()
        self.color = QColor(color)
        self.setFixedSize(14, 14)

    def set_color(self, color):
        self.color = QColor(color)
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(Qt.NoPen)
        painter.setBrush(self.color)
        painter.drawEllipse(2, 2, 10, 10)


class StatusBadge(QWidget):
    def __init__(self, title):
        super().__init__()
        row = QHBoxLayout(self)
        row.setContentsMargins(6, 2, 6, 2)
        row.setSpacing(5)
        self.dot = Dot()
        self.label = QLabel(title)
        self.label.setObjectName("badge")
        row.addWidget(self.dot)
        row.addWidget(self.label)

    def update_state(self, title, color):
        self.label.setText(title)
        self.dot.set_color(color)


class GuiNode(Node):
    def __init__(self, window):
        super().__init__("eva_gui")
        self.window = window
        transient = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                               reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.command_pub = self.create_publisher(String, "/eva/command/input", 10)
        for topic, callback in (
            ("/eva/gnss/status", window.on_gnss_status),
            ("/eva/flex/status", window.on_flex_status),
            ("/eva/voice/status", window.on_voice_status),
            ("/eva/command/status", window.on_command_status),
            ("/eva/system/status", window.on_system_status),
        ):
            self.create_subscription(String, topic, lambda msg, cb=callback: cb(decode(msg.data)), transient)
        self.create_subscription(NavSatFix, "/eva/gnss/fix", window.on_fix, 10)
        self.create_subscription(String, "/eva/voice/event",
                                 lambda msg: window.on_voice_event(decode(msg.data)), 10)
        self.create_subscription(String, "/eva/flex/raw",
                                 lambda msg: window.on_flex_data(decode(msg.data)), 10)
        self.create_subscription(String, "/rover/gesture/event",
                                 lambda msg: window.on_gesture(decode(msg.data)), 10)
        self.create_subscription(String, "/rover/vision/target",
                                 lambda msg: window.on_target(decode(msg.data)), 1)
        self.create_subscription(String, "/rover/follow/guidance",
                                 lambda msg: window.on_guidance(decode(msg.data)), 1)

    def command(self, name):
        msg = String()
        msg.data = json.dumps({"source": "TOUCH", "command": name, "timestamp": time.time()},
                              separators=(",", ":"))
        self.command_pub.publish(msg)


def value_label(caption, initial="--"):
    box = QWidget()
    layout = QVBoxLayout(box)
    layout.setContentsMargins(0, 0, 0, 0)
    layout.setSpacing(1)
    title = QLabel(caption)
    title.setObjectName("caption")
    value = QLabel(initial)
    value.setObjectName("value")
    layout.addWidget(title)
    layout.addWidget(value)
    return box, value


class EVAWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Rover GUI")
        self.setMinimumSize(640, 400)
        self.history = deque(maxlen=8)
        self.rover_online = False
        self.last_system = None
        self.gnss = self.flex = self.voice = {}
        self._build()
        self._style()
        self.clock_timer = QTimer(self)
        self.clock_timer.timeout.connect(self._tick)
        self.clock_timer.start(1000)
        self._tick()

    def _build(self):
        root = QWidget()
        outer = QVBoxLayout(root)
        outer.setContentsMargins(12, 8, 12, 9)
        outer.setSpacing(7)

        header = QHBoxLayout()
        title = QLabel("EVA SUIT // MISSION CONTROL")
        title.setObjectName("title")
        self.clock = QLabel()
        self.clock.setObjectName("clock")
        self.desktop = QPushButton("UBUNTU DESKTOP")
        self.desktop.setObjectName("desktop")
        self.desktop.clicked.connect(self.showMinimized)
        header.addWidget(title)
        header.addStretch()
        header.addWidget(self.clock)
        header.addWidget(self.desktop)
        outer.addLayout(header)

        statuses = QHBoxLayout()
        self.badges = {name: StatusBadge(name) for name in ("GNSS", "FLEX", "MIC", "LINK")}
        for badge in self.badges.values():
            statuses.addWidget(badge)
        statuses.addStretch()
        self.simulation = QLabel("LIVE INPUTS")
        self.simulation.setObjectName("live")
        statuses.addWidget(self.simulation)
        outer.addLayout(statuses)

        self.stack = QStackedWidget()
        self.stack.addWidget(self._dashboard())
        self.stack.addWidget(self._commands())
        self.stack.addWidget(self._system())
        outer.addWidget(self.stack, 1)

        nav = QHBoxLayout()
        for index, name in enumerate(("DASHBOARD", "COMMANDS", "SYSTEM")):
            button = QPushButton(name)
            button.setCheckable(True)
            button.setAutoExclusive(True)
            button.clicked.connect(lambda _checked, page=index: self.stack.setCurrentIndex(page))
            if index == 0:
                button.setChecked(True)
            nav.addWidget(button)
        self.fullscreen_button = QPushButton("ROVER GUI")
        self.fullscreen_button.setObjectName("restore")
        self.fullscreen_button.clicked.connect(self.restore_fullscreen)
        nav.addWidget(self.fullscreen_button)
        outer.addLayout(nav)
        self.setCentralWidget(root)

    def _dashboard(self):
        page = QFrame()
        page.setObjectName("panel")
        grid = QGridLayout(page)
        grid.setContentsMargins(14, 9, 14, 9)
        grid.setSpacing(5)
        fields = [
            ("GPS FIX", "fix_value"), ("SATELLITES", "sat_value"),
            ("LATITUDE", "lat_value"), ("LONGITUDE", "lon_value"),
            ("ALTITUDE", "alt_value"), ("ROVER MODE", "mode_value"),
        ]
        for index, (caption, attr) in enumerate(fields):
            box, label = value_label(caption)
            setattr(self, attr, label)
            grid.addWidget(box, index // 2, index % 2)
        voice_box, self.voice_value = value_label("VOICE", "INITIALIZING")
        command_box, self.command_value = value_label("LAST COMMAND", "NONE")
        result_box, self.result_value = value_label("RESULT", "NO COMMAND SENT")
        grid.addWidget(voice_box, 3, 0, 1, 2)
        grid.addWidget(command_box, 4, 0)
        grid.addWidget(result_box, 4, 1)
        event_box, self.event_value = value_label("LATEST MISSION EVENT", "NONE")
        grid.addWidget(event_box, 5, 0, 1, 2)
        return page

    def _commands(self):
        page = QFrame()
        page.setObjectName("panel")
        layout = QGridLayout(page)
        layout.setContentsMargins(14, 12, 14, 12)
        self.command_buttons = {}
        for index, name in enumerate(("COME", "FOLLOW", "FETCH", "GIVE", "STOP")):
            button = QPushButton(name)
            button.setObjectName("stop" if name == "STOP" else "command")
            button.setMinimumHeight(68)
            button.clicked.connect(lambda _checked, command=name: self.send_command(command))
            if name in ("FETCH", "GIVE"):
                button.setEnabled(False)
                button.setText(f"{name}\nUNAVAILABLE")
            self.command_buttons[name] = button
            layout.addWidget(button, index // 3, index % 3)
        warning = QLabel("COMMANDS REQUIRE A LIVE ROVER LINK AND ROVER ACKNOWLEDGEMENT\n"
                         "TOUCHSCREEN STOP DOES NOT REPLACE THE PHYSICAL E-STOP")
        warning.setObjectName("warning")
        warning.setAlignment(Qt.AlignCenter)
        layout.addWidget(warning, 2, 0, 1, 3)
        self._set_command_availability(False)
        return page

    def _system(self):
        page = QFrame()
        page.setObjectName("panel")
        layout = QGridLayout(page)
        layout.setContentsMargins(14, 9, 14, 9)
        items = [
            ("BACKEND", "backend_value"), ("ROS / ROVER LINK", "link_value"),
            ("GNSS DEVICE", "gnss_system"), ("FLEX DEVICE", "flex_system"),
            ("MICROPHONE", "mic_system"), ("WHISPER", "whisper_system"),
            ("ACK RECEIVER", "ack_system"), ("LAST ERROR", "error_system"),
        ]
        for index, (caption, attr) in enumerate(items):
            box, label = value_label(caption)
            label.setWordWrap(True)
            setattr(self, attr, label)
            layout.addWidget(box, index // 2, index % 2)
        return page

    def _style(self):
        self.setStyleSheet(f"""
            QWidget {{ background:#090d10; color:#dce8e6; font-family:'DejaVu Sans Mono'; }}
            QLabel#title {{ color:{GREEN}; font-size:18px; font-weight:700; letter-spacing:1px; }}
            QLabel#clock {{ color:{CYAN}; font-size:16px; }}
            QLabel#badge {{ color:#aab8ba; font-size:12px; font-weight:600; }}
            QLabel#live {{ color:{GREEN}; border:1px solid #27563a; padding:3px 7px; }}
            QFrame#panel {{ background:#0d1418; border:1px solid #24404a; }}
            QLabel#caption {{ color:#6e9ca5; font-size:10px; letter-spacing:1px; }}
            QLabel#value {{ color:#ecfafa; font-size:17px; font-weight:600; }}
            QLabel#warning {{ color:{AMBER}; font-size:10px; }}
            QPushButton {{ background:#111c21; color:{CYAN}; border:1px solid #2b5662;
                           padding:8px; font-size:12px; font-weight:700; }}
            QPushButton:pressed, QPushButton:checked {{ background:#173b34; color:{GREEN};
                                                       border-color:{GREEN}; }}
            QPushButton:disabled {{ color:#566067; border-color:#293136; background:#101416; }}
            QPushButton#desktop {{ color:{AMBER}; border-color:#6b5530; padding:6px 9px; }}
            QPushButton#restore {{ color:{GREEN}; border-color:#2f7050; }}
            QPushButton#command {{ font-size:18px; }}
            QPushButton#stop {{ color:{RED}; border-color:#7b3340; font-size:20px; }}
        """)

    def _tick(self):
        self.clock.setText(QDateTime.currentDateTime().toString("HH:mm:ss"))
        if self.last_system is not None and time.monotonic() - self.last_system > 2.0:
            self.rover_online = False
            self.backend_value.setText("OFFLINE / STALE")
            self.link_value.setText("DISCONNECTED")
            self.badges["LINK"].update_state("LINK", RED)
            self._set_command_availability(False)

    def restore_fullscreen(self):
        self.show()
        self.showFullScreen()
        self.raise_()
        self.activateWindow()

    def _set_command_availability(self, online):
        for name in ("COME", "FOLLOW", "STOP"):
            if hasattr(self, "command_buttons"):
                self.command_buttons[name].setEnabled(online)

    def send_command(self, command):
        if not self.rover_online:
            self.result_value.setText("REJECTED: ROVER DISCONNECTED")
            return
        self.command_value.setText(command)
        self.result_value.setText("SUBMITTING TO BACKEND")
        self.node.command(command)

    def on_fix(self, msg):
        valid = msg.status.status >= 0 and math.isfinite(msg.latitude) and math.isfinite(msg.longitude)
        self.lat_value.setText(f"{msg.latitude:.7f}" if valid else "--")
        self.lon_value.setText(f"{msg.longitude:.7f}" if valid else "--")
        self.alt_value.setText(f"{msg.altitude:.2f} m" if math.isfinite(msg.altitude) else "--")

    def on_gnss_status(self, data):
        self.gnss = data
        connected, valid = data.get("connected", False), data.get("fix_valid", False)
        color = GREEN if valid else AMBER if connected else RED
        self.badges["GNSS"].update_state("GNSS", color)
        self.fix_value.setText(str(data.get("fix_type", data.get("state", "--"))))
        satellites = data.get("satellites")
        self.sat_value.setText("--" if satellites is None else str(satellites))
        self.gnss_system.setText(f"{data.get('state', '--')} // {data.get('port', 'NO PORT')}")

    def on_flex_status(self, data):
        self.flex = data
        state = data.get("state", "UNKNOWN")
        color = GREEN if state == "OK" else AMBER if data.get("connected") else RED
        self.badges["FLEX"].update_state("FLEX", color)
        self.flex_system.setText(f"{state} // {data.get('port', 'NO PORT')}")

    def on_voice_status(self, data):
        self.voice = data
        state = data.get("state", "UNKNOWN")
        color = GREEN if state in ("LISTENING", "PROCESSING") else GRAY if state == "DISABLED" else RED
        self.badges["MIC"].update_state("MIC", color)
        self.voice_value.setText(state)
        mic = "CONNECTED" if data.get("microphone_connected") else "DISCONNECTED"
        self.mic_system.setText(f"{mic} // {data.get('device', 'NO DEVICE')}")
        self.whisper_system.setText("READY" if data.get("whisper_loaded") else "NOT LOADED")

    def on_system_status(self, data):
        self.last_system = time.monotonic()
        self.rover_online = bool(data.get("rover_connected"))
        self.badges["LINK"].update_state("LINK", GREEN if self.rover_online else RED)
        self.mode_value.setText(str(data.get("rover_state", "UNKNOWN")))
        host = data.get("host") or {}
        metrics = [str(data.get("backend", "UNKNOWN")), str(host.get("architecture", "?"))]
        if host.get("temperature_c") is not None:
            metrics.append(f"{host['temperature_c']} C")
        if host.get("memory_available_mib") is not None:
            metrics.append(f"{host['memory_available_mib']} MiB FREE")
        self.backend_value.setText(" // ".join(metrics))
        self.link_value.setText("CONNECTED" if self.rover_online else "DISCONNECTED")
        self.ack_system.setText("AVAILABLE" if data.get("ack_available") else "UNAVAILABLE")
        errors = data.get("last_errors") or []
        self.error_system.setText(str(errors[-1]) if errors else "NONE")
        self._set_command_availability(self.rover_online)
        if data.get("simulation"):
            self.simulation.setText("SIMULATION")
            self.simulation.setStyleSheet(f"color:{AMBER}")

    def on_command_status(self, data):
        command = data.get("command") or "UNKNOWN"
        self.command_value.setText(command)
        status, reason = data.get("status", "UNKNOWN"), data.get("reason", "")
        self.result_value.setText(f"{status}: {reason}".rstrip(": "))
        self._event(f"{command} // {status}")

    def on_voice_event(self, data):
        phrase = data.get("phrase") or "(no speech)"
        self.voice_value.setText(f"{phrase[:44]} // {data.get('reason', '')}")
        if data.get("command"):
            self._event(f"VOICE {data['command']} // {data.get('delivery', 'LOCAL')}")

    def on_flex_data(self, data):
        self._event(f"FLEX RAW {data.get('flex_raw')} // SEQ {data.get('seq')}")

    def on_gesture(self, data):
        if data.get("gesture") == "COME":
            self._event(f"ZED COME // PERSON {data.get('person_id', '--')}")

    def on_target(self, data):
        if data.get("locked") and data.get("valid"):
            self._event(f"PERSON LOCKED // ID {data.get('person_id', '--')}")

    def on_guidance(self, data):
        if data.get("state"):
            self.mode_value.setText(str(data["state"]))
            if data["state"] == "FOLLOW":
                self._event(f"FOLLOW // {data.get('reason', 'UNKNOWN')}")

    def _event(self, text):
        self.history.appendleft(text)
        self.event_value.setText(text)


def acquire_single_instance(name="batmobile_eva_rover_gui"):
    probe = QLocalSocket()
    probe.connectToServer(name)
    if probe.waitForConnected(250):
        probe.write(b"RESTORE")
        probe.waitForBytesWritten(250)
        return None
    QLocalServer.removeServer(name)
    server = QLocalServer()
    if not server.listen(name):
        raise RuntimeError(f"cannot create GUI singleton socket: {server.errorString()}")
    return server


def main(args=None):
    parser = argparse.ArgumentParser(description="Batmobile EVA touchscreen GUI")
    parser.add_argument("--config", default="", help="EVA YAML configuration path")
    source_args = sys.argv[1:] if args is None else list(args)
    known, ros_args = parser.parse_known_args(source_args)
    config_path = known.config or os.path.join(
        get_package_share_directory("eva_suit"), "config", "eva.yaml")
    with open(config_path, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    domain = config.get("ros", {}).get("domain_id")
    if domain is not None:
        domain = int(domain)
        if not 0 <= domain <= 232:
            raise ValueError("ROS domain_id must be between 0 and 232")
        os.environ["ROS_DOMAIN_ID"] = str(domain)
    os.environ["ROS_LOCALHOST_ONLY"] = "1" if config.get("ros", {}).get("localhost_only") else "0"
    rclpy.init(args=ros_args)
    app = QApplication([sys.argv[0]])
    server = acquire_single_instance()
    if server is None:
        rclpy.shutdown()
        return
    window = EVAWindow()
    node = GuiNode(window)
    window.node = node

    def restore_connection():
        socket = server.nextPendingConnection()
        socket.waitForReadyRead(100)
        socket.readAll()
        window.restore_fullscreen()
        socket.disconnectFromServer()

    server.newConnection.connect(restore_connection)
    spin_timer = QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    spin_timer.start(20)
    window.showFullScreen()
    code = app.exec_()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
