import os
import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String


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


def detect_teensy(teensy_serials):
    for teensy_serial in teensy_serials:
        try:
            serial_port = get_tty_from_serial(teensy_serial)
            if serial_port is not None:
                print(f"Found Teensy {teensy_serial} at {serial_port}")
                return teensy_serial, serial_port
        except Exception as e:
            print(f"Error checking Teensy {teensy_serial}: {e}")
            continue
    return None, None


class BifrostNode(Node):

    def __init__(self, detected_serial: str, detected_port: str):
        super().__init__("bifrost_node")

        self.declare_parameter("teensy_serial", detected_serial)
        self.declare_parameter("serial_port", detected_port)
        self.declare_parameter("baud_rate", 115200)

        teensy_serial = self.get_parameter("teensy_serial").get_parameter_value().string_value
        self.serial_port = self.get_parameter("serial_port").get_parameter_value().string_value
        self.baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value

        self.serial_conn = serial.Serial(self.serial_port, self.baud_rate)
        self.subscription = self.create_subscription(String, "/science/keypad", self.keypad_callback, 10)
        self.get_logger().info(f"BifrostNode started | teensy_serial={teensy_serial} | port={self.serial_port}")
        self.ENABLE = None

    def keypad_callback(self, msg: String):
        self.get_logger().info(f"[/science/keypad] data='{msg.data}'")
        data = msg.data.strip()

        if data == "ENABLE_ON":
            self.ENABLE = True
        elif data == "ENABLE_OFF":
            self.ENABLE = False

        if self.ENABLE:
            if data == "Power_ON":
                self.serial_conn.write(b'd\n')
            elif data == "Power_OFF":
                self.serial_conn.write(b'p\n')
            elif data == "DrillerServo_ON":
                self.serial_conn.write(b't\n')
            elif data == "DrillerServo_OFF":
                self.serial_conn.write(b'z\n')
            elif data in ("Fire_ON", "Fire_OFF"):
                self.serial_conn.write(b'm\n')
            elif data in ("BifrostServoON", "BifrostServoOFF"):
                self.serial_conn.write(b's\n')
            elif data in ("HeatModuleSideRight", "HeatModuleSideLeft"):
                self.serial_conn.write(b'c\n')
            elif data == "DrillerUp":
                self.serial_conn.write(b'a\n')
            elif data == "DrillerUpStop":
                self.serial_conn.write(b'w\n')
            elif data == "DrillerDown":
                self.serial_conn.write(b'b\n')
            elif data == "DrillerDownStop":
                self.serial_conn.write(b'w\n')
            elif data == "RotateCameraLeft":
                self.serial_conn.write(b'1\n')
            elif data in ("RotateCameraLeftStop", "RotateCameraRightStop"):
                self.serial_conn.write(b'0\n')
            elif data == "RotateCameraRight":
                self.serial_conn.write(b'-1\n')
            elif data in ("LED_ON", "LED_OFF"):
                self.serial_conn.write(b'u\n')
            else:
                self.get_logger().warn(f"Unknown keypad command: {data}")


def main(args=None):
    
    teensy_serials = ["16832280"]
    detected_serial, detected_port = detect_teensy(teensy_serials)
    if detected_serial is None:
        print("No Teensy found! Checked: " + ", ".join(teensy_serials))
        return

    rclpy.init(args=args)
    node = BifrostNode(detected_serial=detected_serial, detected_port=detected_port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()