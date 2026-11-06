import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Header

import serial
import serial.tools.list_ports
import struct
import math
import threading
import time
import pynmea2

# =========================================================================
# 1. MATH & PROTOCOL HELPERS (Strictly Copied from Original)
# =========================================================================
FRAME_HEAD = 0xFC
FRAME_END  = 0xFD
TYPE_IMU   = 0x40
TYPE_AHRS  = 0x41
PI = math.pi

def quat_mul(q1, q2):
    """Standard Hamilton Product"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    )

def quat_to_roll_pitch(q):
    w, x, y, z = q
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    sinp = 2*(w*y - z*x)
    pitch = math.asin(max(-1, min(1, sinp)))
    return roll, pitch

def tilt_compensated_heading(roll, pitch, mx, my, mz):
    """
    Returns heading in degrees [0, 360).
    Logic strictly preserved from user provided code.
    """
    t1 = my*math.cos(roll) + mz*math.sin(roll)
    t2 = mx*math.cos(pitch) + my*math.sin(pitch)*math.sin(roll) - mz*math.sin(pitch)*math.cos(roll)
    yaw = math.atan2(-t1, t2)
    if yaw < 0:
        yaw += 2*math.pi
    return (360 - math.degrees(yaw)) % 360

def read_exact(ser, n):
    buf = b''
    while len(buf) < n:
        c = ser.read(n-len(buf))
        if not c:
            return None
        buf += c
    return buf

# =========================================================================
# 2. IMU DRIVER CLASS
# =========================================================================
class N100HeadingDriver:
    def __init__(self, parent_node, port, baud):
        self.node = parent_node
        self.lock = threading.Lock()
        self.latest = {}
        
        # Coordinate rotation quaternions (Preserved)
        self.q_rr = (math.cos(PI/2), math.sin(PI/2), 0, 0)
        self.q_z  = (math.cos(PI/2), 0, 0, math.sin(PI/2))

        # Create Publisher
        self.pub = self.node.create_publisher(Imu, "/imu/data", 10)

        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            threading.Thread(target=self.reader, daemon=True).start()
        except Exception as e:
            self.node.get_logger().error(f"N100 Connection Failed: {e}")
            raise e

    def reader(self):
        # defined OUTSIDE the loop to persist data between packets
        last = {} 
        
        while rclpy.ok():
            try:
                # 1. Read Header
                h = read_exact(self.ser, 1)
                if not h or h[0] != FRAME_HEAD:
                    continue

                # 2. Read Length
                hdr = read_exact(self.ser, 6)
                if not hdr: continue
                
                payload_len = hdr[1]
                
                # 3. Read Payload + End Byte
                payload_end = read_exact(self.ser, payload_len + 1)
                if not payload_end or payload_end[-1] != FRAME_END:
                    continue

                payload = payload_end[:-1]

                # -------- IMU PACKET (Magnetometer) --------
                if hdr[0] == TYPE_IMU:
                    v = struct.unpack("<12fq", payload)
                    # Logic Check: Indices 6,7,8 are mx, my, mz
                    last["mx"] = v[6] * 1e-7
                    last["my"] = v[7] * 1e-7
                    last["mz"] = v[8] * 1e-7

                # -------- AHRS PACKET (Orientation) --------
                elif hdr[0] == TYPE_AHRS:
                    v = struct.unpack("<10fq", payload)
                    # Logic Check: Indices 6,7,8,9 are qw, qx, qy, qz
                    qw, qx, qy, qz = v[6:10]
                    raw_q = (qw, qx, qy, qz)
                    
                    # Logic Check: Apply Coordinate Transforms
                    q = quat_mul(self.q_z, quat_mul(self.q_rr, raw_q))
                    
                    last["roll"], last["pitch"] = quat_to_roll_pitch(q)
                    last["q"] = q

                # -------- CALCULATE HEADING --------
                # Only runs if we have BOTH mag data and orientation data
                if all(k in last for k in ("roll", "pitch", "mx", "my", "mz", "q")):
                    heading = tilt_compensated_heading(
                        last["roll"], last["pitch"],
                        last["mx"], last["my"], last["mz"]
                    )
                    
                    with self.lock:
                        self.latest = {
                            "heading": heading,
                            "q": last["q"],
                            "time": time.time()
                        }
                        
            except Exception:
                # Ignore serial hiccups to keep thread alive
                pass 

    def update_and_publish(self):
        """Called by the main node timer to publish ROS messages"""
        with self.lock:
            if not self.latest:
                return None
            d = self.latest.copy()

        # ---- Publish ROS Message ----
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        qw, qx, qy, qz = d["q"]
        msg.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        # Standard covariance for orientation known / others unknown
        msg.orientation_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
        msg.angular_velocity_covariance = [-1.0] * 9
        msg.linear_acceleration_covariance = [-1.0] * 9

        self.pub.publish(msg)
        return d["heading"]

# =========================================================================
# 3. MAIN NODE (GNSS + IMU)
# =========================================================================
class GNSSIMUNode(Node):
    def __init__(self):
        super().__init__('gnss_imu_combined_node')
        self.get_logger().info('Starting GNSS + IMU combined node')

        # # --- IMU Setup ---
        # imu_vid = 6790
        # self.imu_port = self.find_serial_device_by_vid_pid(imu_vid)
        self.imu = None
        # self.altitude = 0.0

        # if self.imu_port:
        #     try:
        #         # 921600 is required for N100
        #         self.imu = N100HeadingDriver(self, self.imu_port, 921600)
        #         self.get_logger().info(f'IMU connected: {self.imu_port}')
        #     except Exception as e:
        #         self.get_logger().error(f'Failed to init IMU: {e}')
        # else:
        #     self.get_logger().error('IMU port not found (VID=4292)')

        # --- GPS Setup ---
        gps_vid = 5446
        self.gps_port = self.find_serial_device_by_vid_pid(gps_vid)
        self.gps_ser = None

        if self.gps_port:
            try:
                self.gps_ser = serial.Serial(self.gps_port, 115200, timeout=0.1)
                self.get_logger().info(f'GPS connected: {self.gps_port}')
            except Exception as e:
                self.get_logger().error(f'Failed to open GPS serial: {e}')
        else:
            self.get_logger().error('GPS port not found (VID=5446)')

        self.nav_sat_pub = self.create_publisher(NavSatFix, '/nav_sat', 10)
        self.create_timer(0.1, self.update_sensors)

    def find_serial_device_by_vid_pid(self, vid):
        for port in serial.tools.list_ports.comports():
            if port.vid == vid:
                return port.device
        return None

    def update_sensors(self):
        # 1. Process IMU (Publishes inside the driver class)
        if self.imu:
            heading = self.imu.update_and_publish()
            if heading is not None:
                self.get_logger().debug(f'IMU Heading: {heading:.2f}°')

        # 2. Process GPS
        if self.gps_ser:
            self.process_gps()

    def process_gps(self):
        try:
            while self.gps_ser.in_waiting > 0:
                line = self.gps_ser.readline()
                if not line.startswith((b'$GNGGA', b'$GNRMC', b'$GPGGA', b'$GPRMC')):
                    continue
                try:
                    nmea_str = line.decode('utf-8', errors='ignore').strip()
                    gps_data = pynmea2.parse(nmea_str)
                    if not hasattr(gps_data, 'latitude') or float(gps_data.latitude) == 0.0:
                        continue
                    
                    if hasattr(gps_data, 'altitude'):
                        self.altitude = float(gps_data.altitude)
                        # print(f'Altitude: {self.altitude} m')

    

                    msg = NavSatFix()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'gps_link'
                    msg.latitude = float(gps_data.latitude)
                    msg.longitude = float(gps_data.longitude)
                    msg.altitude = self.altitude
                    self.nav_sat_pub.publish(msg)
                    self.get_logger().info(f'{msg.latitude}, {msg.longitude}, {msg.altitude}m')
                except:
                    pass
        except Exception as e:
            self.get_logger().warn(f'GPS error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = GNSSIMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
