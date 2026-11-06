import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial.tools.list_ports

class SensorDataPublisher(Node):
    def __init__(self):
        super().__init__('sensor_data_publisher')
        
        # Open serial port
        vid = 6790  # Replace with your device's VID
        baud = 9600
        port = self.find_serial_device_by_vid_pid(vid)
        if port is None:
            self.get_logger().error(f"No device found with VID: {vid}")
            exit(1)

        self.ser = serial.Serial(port, baud)  # Adjust '/dev/ttyACM0' to match your setup
        self.sensor_data_pub = self.create_publisher(String, '/sensor_data', 10)
        # Start reading data
        # self.read_data()

        self.timer = self.create_timer(0.01, self.read_data)
        
    def find_serial_device_by_vid_pid(self, vid):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.vid == vid:
                return port.device
        return None

    # def read_data(self):
    #     #self.read_data()
    #     while rclpy.ok():
    #         if self.ser.in_waiting > 0:
    #             line = self.ser.readline().decode('utf-8', errors='ignore').strip()
    #             self.publish_data(line)

    def read_data(self):
        # Initialize persistent buffer variables if not already set
        if not hasattr(self, 'buffer'):
            self.buffer = ""
            self.capturing = False

        # Read all available bytes
        while self.ser.in_waiting > 0:
            char = self.ser.read().decode('utf-8', errors='ignore')

            # Start of JSON
            if char == '{':
                self.buffer = char
                self.capturing = True
                continue

            # If capturing, keep building the message
            if self.capturing:
                self.buffer += char

                # End of JSON
                if char == '}':
                    self.publish_data(self.buffer)
                    self.buffer = ""
                    self.capturing = False

    def publish_data(self, data):
        # Create a String message
        sensor_data_msg = String()
        sensor_data_msg.data = data
        
        # Publish the message
        self.sensor_data_pub.publish(sensor_data_msg)
        self.get_logger().info(f"Published: {sensor_data_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    sensor_data_publisher = SensorDataPublisher()
    rclpy.spin(sensor_data_publisher)
    sensor_data_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
