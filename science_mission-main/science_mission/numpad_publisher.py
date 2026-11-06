#!/usr/bin/env python3
"""
Numpad Publisher Node
Reads HID numpad key events and publishes human-readable
command strings (e.g. "Power ON", "LED OFF") to /science/keypad.
"""

import sys
import signal
import select

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from evdev import InputDevice, categorize, ecodes, KeyEvent, list_devices


BUS_USB = 0x03


class NumpadPublisher(Node):
    """ROS2 node that reads a USB numpad and publishes command strings."""

    _REJECT_KEYWORDS = (
        "at translated", "thinkpad", "internal", "laptop",
        "macbook", "chromebook", "built-in",
        "power button", "video bus", "pc speaker", "sleep button",
        "lid switch", "virtual", "sysrq",
        "system control", "consumer control",
    )

    def __init__(self):
        super().__init__("numpad_publisher")

        # --- Parameters ---
        self.declare_parameter("device_path", "")
        self.declare_parameter("grab", True)

        device_path = self.get_parameter("device_path").get_parameter_value().string_value
        self._grab_requested = self.get_parameter("grab").get_parameter_value().bool_value

        # --- Publisher ---
        self.pub = self.create_publisher(String, "/science/keypad", 10)

        # --- Internal state ---
        self._grabbed = False
        self._running = True
        self._ctrl_held = False
        self.device = None

        # Toggle states: Power, DS, BS, HMS, Fire, Light
        self._power = False
        self._ds = False
        self._bs = False
        self._hms = False   # False = Left, True = Right
        self._fire = False
        self._light = False

        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        # --- Detect device ---
        self._device_path = device_path if device_path else self._auto_detect_device()
        if not self._device_path:
            self.get_logger().error("Could not find a USB keypad. Check connection.")
            sys.exit(1)

        self.get_logger().info(f"Using device: {self._device_path}")

    # ------------------------------------------------------------------ #
    #  Device auto-detection (same logic as numpad_input.py)
    # ------------------------------------------------------------------ #
    def _auto_detect_device(self):
        self.get_logger().info("Auto-scanning for USB keypads...")
        candidates = []

        for path in list_devices():
            try:
                dev = InputDevice(path)
            except Exception:
                continue

            name = dev.name
            low = name.lower()
            phys = (dev.phys or "").lower()
            bustype = dev.info.bustype

            if bustype != BUS_USB or "usb" not in phys or ecodes.EV_KEY not in dev.capabilities():
                dev.close()
                continue

            if "mouse" in low or any(kw in low for kw in self._REJECT_KEYWORDS):
                dev.close()
                continue

            score = 10
            if "keypad" in low or "numpad" in low:
                score = 100
            elif "hid" in low:
                score = 50

            candidates.append((score, path, name))
            dev.close()

        if not candidates:
            return None

        candidates.sort(key=lambda c: c[0], reverse=True)
        return candidates[0][1]

    # ------------------------------------------------------------------ #
    #  Signal handling
    # ------------------------------------------------------------------ #
    def _signal_handler(self, signum, frame):
        sig_name = signal.Signals(signum).name
        self.get_logger().info(f"Received {sig_name}. Stopping...")
        self._running = False

    # ------------------------------------------------------------------ #
    #  Publish helper
    # ------------------------------------------------------------------ #
    def _publish_command(self, command: str):
        """Publish a command string and log it."""
        msg = String()
        msg.data = command
        self.pub.publish(msg)
        self.get_logger().info(f"Published: {command}")

    # ------------------------------------------------------------------ #
    #  Main event loop
    # ------------------------------------------------------------------ #
    def run(self):
        try:
            self.device = InputDevice(self._device_path)
            self.get_logger().info(f"Connected to: {self.device.name} ({self._device_path})")

            if self._grab_requested:
                self.device.grab()
                self._grabbed = True
                self.get_logger().info("Device GRABBED (exclusive access).")

            self.get_logger().info("Listening for numpad key presses...")

            fd = self.device.fd

            while self._running and rclpy.ok():
                r, _, _ = select.select([fd], [], [], 0.1)
                if not r:
                    continue

                for event in self.device.read():
                    if event.type != ecodes.EV_KEY:
                        continue

                    key_event = categorize(event)
                    keycode = key_event.keycode
                    if isinstance(keycode, list):
                        keycode = keycode[0]

                    # -- Ctrl tracking for Ctrl+C / Ctrl+Z exit --
                    if keycode in ("KEY_LEFTCTRL", "KEY_RIGHTCTRL"):
                        if key_event.keystate in (KeyEvent.key_down, KeyEvent.key_hold):
                            self._ctrl_held = True
                        elif key_event.keystate == KeyEvent.key_up:
                            self._ctrl_held = False
                        continue

                    if key_event.keystate == KeyEvent.key_down and self._ctrl_held:
                        if keycode in ("KEY_C", "KEY_Z"):
                            self.get_logger().info(f"Ctrl+{keycode[-1]} detected. Exiting...")
                            self._running = False
                            return

                    # -- Only process key-down for toggle/continuous commands --
                    is_press = (key_event.keystate == KeyEvent.key_down)
                    is_release = (key_event.keystate == KeyEvent.key_up)

                    # === TOGGLE KEYS (publish on press) ===
                    if is_press:
                        if keycode in ("KEY_KPSLASH",):
                            self._power = not self._power
                            self._publish_command(f"Power_{'ON' if self._power else 'OFF'}")

                        elif keycode in ("KEY_KPASTERISK",):
                            self._ds = not self._ds
                            self._publish_command(f"DrillerServo_{'ON' if self._ds else 'OFF'}")

                        elif keycode in ("KEY_KPDOT", "KEY_DELETE"):
                            self._bs = not self._bs
                            self._publish_command(f"BifrostServo{'ON' if self._bs else 'OFF'}")

                        elif keycode in ("KEY_KP5", "KEY_CLEAR"):
                            self._light = not self._light
                            self._publish_command(f"LED_{'ON' if self._light else 'OFF'}")

                        elif keycode in ("KEY_KPENTER", "KEY_ENTER"):
                            self._fire = not self._fire
                            self._publish_command(f"Fire_{'ON' if self._fire else 'OFF'}")

                        elif keycode in ("KEY_KP0", "KEY_INSERT"):
                            if not self._fire:
                                self._hms = not self._hms
                                self._publish_command(f"HeatModuleSide{'Right' if self._hms else 'Left'}")
                            else:
                                self.get_logger().warn("HMS blocked (Fire is ON)")

                        # === CONTINUOUS KEYS (publish move on press) ===
                        elif keycode in ("KEY_KP8", "KEY_UP"):
                            self._publish_command("DrillerUp")

                        elif keycode in ("KEY_KP2", "KEY_DOWN"):
                            self._publish_command("DrillerDown")

                        elif keycode in ("KEY_KP4", "KEY_LEFT"):
                            self._publish_command("RotateCameraLeft")

                        elif keycode in ("KEY_KP6", "KEY_RIGHT"):
                            self._publish_command("RotateCameraRight")

                    # === CONTINUOUS KEYS (publish stop on release) ===
                    elif is_release:
                        if keycode in ("KEY_KP8", "KEY_UP"):
                            self._publish_command("DrillerUpStop")

                        elif keycode in ("KEY_KP2", "KEY_DOWN"):
                            self._publish_command("DrillerDownStop")

                        elif keycode in ("KEY_KP4", "KEY_LEFT"):
                            self._publish_command("RotateCameraLeftStop")

                        elif keycode in ("KEY_KP6", "KEY_RIGHT"):
                            self._publish_command("RotateCameraRightStop")

        except PermissionError:
            self.get_logger().error(
                "Permission denied! Add your user to the 'input' group or run with sudo."
            )
        except FileNotFoundError:
            self.get_logger().error(f"Device {self._device_path} not found. Check USB connection.")
        except KeyboardInterrupt:
            pass
        except OSError as e:
            if self._running:
                self.get_logger().error(f"Device error: {e}")
        finally:
            self._cleanup()

    # ------------------------------------------------------------------ #
    #  Cleanup
    # ------------------------------------------------------------------ #
    def _cleanup(self):
        if self.device is not None:
            try:
                if self._grabbed:
                    self.device.ungrab()
                    self._grabbed = False
            except OSError:
                pass
            try:
                self.device.close()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = NumpadPublisher()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
