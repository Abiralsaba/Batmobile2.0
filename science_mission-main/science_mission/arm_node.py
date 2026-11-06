#!/usr/bin/env python3
import os
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial

def get_tty_from_serial(serial_number):
    def find_tty_in_device_tree(device_path):
        try:
            for subdir in os.listdir(device_path):
                if ":" in subdir:
                    tty_class_path = os.path.join(device_path, subdir, "tty")
                    if os.path.exists(tty_class_path):
                        if tty_names := os.listdir(tty_class_path):
                            return f"/dev/{tty_names[0]}"
        except (OSError, IOError):
            pass
        return None

    base_path = "/sys/bus/usb/devices/"
    for device_dir in os.listdir(base_path):
        device_path = os.path.join(base_path, device_dir)
        serial_file = os.path.join(device_path, "serial")
        if os.path.exists(serial_file):
            try:
                with open(serial_file, 'r') as f:
                    if f.read().strip() == serial_number:
                        return find_tty_in_device_tree(device_path)
            except (OSError, IOError):
                continue
    return None

# ── Config ────────────────────────────────────────────────────────────────────
SERIAL_PORT = None
BAUD        = 115200

teensy_serials = ["16832450", "16832510"]  # primary and spare (hot spare)

# Try each teensy until one is found
for teensy_serial in teensy_serials:
    try:
        SERIAL_PORT = get_tty_from_serial(teensy_serial)
        if SERIAL_PORT is not None:
            print(f"Found Teensy {teensy_serial} at {SERIAL_PORT}")
            break
    except Exception as e:
        print(f"Error checking Teensy {teensy_serial}: {e}")
        continue

if SERIAL_PORT is None:
    print("No Teensy found! Checked: " + ", ".join(teensy_serials))
    exit(1)


SEND_HZ     = 50
DEADZONE    = 0.10
EXPO        = 1.6
DT_CLAMP    = 0.05   # guards against scheduler spikes inflating dt

MAX_VEL     = [0.3, 0.8, 0.8, 1.0]

Q_MIN = [-7.0, -2.0,  0.1, -4.14]
Q_MAX = [ 7.0, 20.0, 15.0,  4.14]

BASE_JOG_SPEED = 0.8

# ── DS4 axis / button indices (standard ROS joy_node / kernel HID mapping) ───
#
#   Index  Physical axis
#   -----  ----------------------------------------
#     0    Left  stick X   (+1 = right)
#     1    Left  stick Y   (+1 = up)          ← ROS convention: up is POSITIVE
#     2    L2 trigger      (+1 = unpressed, -1 = fully pressed)
#     3    Right stick X   (+1 = right)
#     4    Right stick Y   (+1 = up)          ← ROS convention: up is POSITIVE
#     5    R2 trigger      (+1 = unpressed, -1 = fully pressed)
#     6    D-pad X         (+1 = left, -1 = right)
#     7    D-pad Y         (+1 = up,   -1 = down)
#
AXIS_LX     = 0
AXIS_LY     = 1
AXIS_RX     = 3
AXIS_RY     = 4
AXIS_DPAD_X = 6
AXIS_DPAD_Y = 7

BTN_CROSS    = 0   # enable
BTN_CIRCLE   = 1   # servo toggle
BTN_TRIANGLE = 2   # hold
BTN_L1       = 4
BTN_R1       = 5


# ── Helpers ───────────────────────────────────────────────────────────────────
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def dz(x, deadzone):
    if abs(x) < deadzone:
        return 0.0
    s = 1.0 if x >= 0 else -1.0
    return s * clamp((abs(x) - deadzone) / (1.0 - deadzone), 0.0, 1.0)

def expo(x, e):
    s = 1.0 if x >= 0 else -1.0
    return s * (abs(x) ** e)

def fmt(q, grip1, grip2, servo):
    return (
        f"{q[0]:.4f},{q[1]:.4f},{q[2]:.4f},{q[3]:.4f},"
        f"{int(grip1)},{int(grip2)},{int(servo)}\n"
    )


# ── ROS2 node ─────────────────────────────────────────────────────────────────
class ArmJoyController(Node):
    def __init__(self):
        super().__init__("arm_joint_teleop")

        self.state = {
            "lx": 0.0, "ly": 0.0,
            "rx": 0.0, "ry": 0.0,
            "dpad_x": 0.0, "dpad_y": 0.0,
            "btn_cross":    0,
            "btn_circle":   0,
            "btn_triangle": 0,
            "btn_l1":       0,
            "btn_r1":       0,
        }

        self.subscription = self.create_subscription(
            Joy, "/joy/arm", self.joy_callback, 10
        )
        self.get_logger().info("Subscribed to /joy/arm")

    def joy_callback(self, msg: Joy):
        axes    = list(msg.axes)
        buttons = list(msg.buttons)

        def ga(i, default=0.0):
            return axes[i] if i < len(axes) else default

        def gb(i, default=0):
            return buttons[i] if i < len(buttons) else default

        self.state.update({
            "lx":           ga(AXIS_LX),
            "ly":           ga(AXIS_LY),
            "rx":           ga(AXIS_RX),
            "ry":           ga(AXIS_RY),
            "dpad_x":       ga(AXIS_DPAD_X),
            "dpad_y":       ga(AXIS_DPAD_Y),
            "btn_cross":    gb(BTN_CROSS),
            "btn_circle":   gb(BTN_CIRCLE),
            "btn_triangle": gb(BTN_TRIANGLE),
            "btn_l1":       gb(BTN_L1),
            "btn_r1":       gb(BTN_R1),
        })


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = ArmJoyController()

    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.0, write_timeout=0.2)
    print(f"Serial connected: {SERIAL_PORT}\n")
    time.sleep(0.5)
    ser.write(b"EN\n") # dummy garbage data
    time.sleep(3)

    print("Controls:")
    print("  Cross (X)     = enable control")
    print("  Circle (O)    = toggle servo 0/1")
    print("  Triangle      = hold position")
    print("  D-pad L/R     = roll")
    print("  D-pad U/D     = gripper 2")
    print("  L1 / R1       = gripper 1")
    print("  Left stick Y  = shoulder")
    print("  Right stick Y = elbow")
    print("  Right stick X = base\n")
    print("  Press X to enable Motors")

    q           = [0.0, 0.0, 0.0, 0.0]
    grip1       = 0
    grip2       = 0
    servo       = 0
    enabled     = False
    prev_circle = 0

    last_t    = time.time()
    last_send = time.time()
    send_period = 1.0 / SEND_HZ

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)

            now = time.time()
            dt  = min(now - last_t, DT_CLAMP)
            last_t = now

            st = node.state

            # ── Enable (rising edge on Cross) ────────────────────────────────
            # if st["btn_cross"] == 1 and not enabled:
            if st["btn_cross"] == 1:
                ser.write(b"x\n")
                enabled = True
                print("Enabled")
                time.sleep(3.0)

            # ── Hold = Triangle held ─────────────────────────────────────────
            hold = (st["btn_triangle"] == 1)

            # ── Servo toggle (rising edge on Circle) ─────────────────────────
            if st["btn_circle"] == 1 and prev_circle == 0:
                servo = 1 - servo
                print(f"Servo: {servo}")
            prev_circle = st["btn_circle"]

            # ── Gripper 1 (L1 / R1) ─────────────────────────────────────────
            if st["btn_r1"] == 1 and st["btn_l1"] == 0:
                grip1 = 1
            elif st["btn_l1"] == 1 and st["btn_r1"] == 0:
                grip1 = -1
            else:
                grip1 = 0

            # ── Gripper 2 (D-pad U/D) ────────────────────────────────────────
            # ROS joy: dpad_y  +1 = up,  -1 = down
            if st["dpad_y"] > 0.5:
                grip2 = 1
            elif st["dpad_y"] < -0.5:
                grip2 = -1
            else:
                grip2 = 0

            # ── Joint motion ─────────────────────────────────────────────────
            if enabled and not hold:
                # ROS joy_node convention: stick UP = +1, stick RIGHT = +1
                ly_v = expo(dz( st["ly"], DEADZONE), EXPO)   # push up    → shoulder +
                ry_v = expo(dz( st["ry"], DEADZONE), EXPO)   # push up    → elbow    +
                rx_v = expo(-dz( st["rx"], DEADZONE), EXPO)   # invert push right → base     +

                # D-pad L/R → roll  (ROS joy: dpad_x +1 = left, -1 = right)
                dpad_x = st["dpad_x"]
                if dpad_x > 0.5:
                    roll_dir = -1.0
                elif dpad_x < -0.5:
                    roll_dir = +1.0
                else:
                    roll_dir = 0.0

                q[0] = clamp(q[0] + rx_v     * MAX_VEL[0]     * dt, Q_MIN[0], Q_MAX[0])  # base     ← right stick X
                q[1] = clamp(q[1] + ly_v     * MAX_VEL[1]     * dt, Q_MIN[1], Q_MAX[1])  # shoulder ← left  stick Y
                q[2] = clamp(q[2] + ry_v     * MAX_VEL[2]     * dt, Q_MIN[2], Q_MAX[2])  # elbow    ← right stick Y
                q[3] = clamp(q[3] + roll_dir * BASE_JOG_SPEED * dt, Q_MIN[3], Q_MAX[3])  # roll     ← D-pad L/R

            # ── Send over serial ─────────────────────────────────────────────
            if enabled and (now - last_send) >= send_period:
                last_send = now
                line = fmt(q, grip1, grip2, servo)
                ser.write(line.encode("utf-8"))
                print(f"SEND: {line.strip()}")

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nExit")
    finally:
        try:
            ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()