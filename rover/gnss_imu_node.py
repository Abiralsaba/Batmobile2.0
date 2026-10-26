"""Real N100 + NMEA driver restored from the operator's original Python source.

Retains both original classes, packet layouts, magnetometer scale and rotations.
FC + six header bytes + payload + FD; IMU <12fq, AHRS <10fq. The supplied
source does not define/check header bytes 3..6 (sequence/CRC in FDILink variants).
CRC validation and the exact device firmware remain UNVERIFIED; never infer them.
ROS stamps are receipt times, not synchronized device capture times.
"""
from datetime import datetime, timedelta, timezone
import math
import re
import struct
import threading
import time

import pynmea2
import rclpy
from geometry_msgs.msg import Quaternion
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import serial
from serial.tools import list_ports
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus


FRAME_HEAD = 0xFC
FRAME_END = 0xFD
TYPE_IMU = 0x40
TYPE_AHRS = 0x41
PI = math.pi
PAYLOAD_FORMATS = {TYPE_IMU: '<12fq', TYPE_AHRS: '<10fq'}


def quat_mul(q1, q2):
    """Original Hamilton product; internal component order is (w, x, y, z)."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    )


def quat_to_roll_pitch(q):
    w, x, y, z = q
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    sinp = 2*(w*y - z*x)
    return roll, math.asin(max(-1, min(1, sinp)))


def tilt_compensated_heading(roll, pitch, mx, my, mz):
    """Original [0, 360) magnetic calculation, without geographic declination.

    At zero tilt: +mag X -> 0 degrees, +mag Y -> 90 degrees. North/clockwise
    correspondence depends on the original sensor axes and mounting, unverified.
    """
    t1 = my*math.cos(roll) + mz*math.sin(roll)
    t2 = (mx*math.cos(pitch) + my*math.sin(pitch)*math.sin(roll)
          - mz*math.sin(pitch)*math.cos(roll))
    yaw = math.atan2(-t1, t2)
    if yaw < 0:
        yaw += 2*math.pi
    return (360 - math.degrees(yaw)) % 360


def extract_packets(buffer):
    """Consume complete original-format packets; retain partial serial reads.

    Reject unexpected type/length/end marker and resynchronize at the next FC.
    This validates framing, NOT CRC integrity. Callers bound read size/timeouts.
    """
    packets = []
    while buffer:
        start = buffer.find(bytes([FRAME_HEAD]))
        if start < 0:
            buffer.clear()
            break
        del buffer[:start]
        if len(buffer) < 7:
            break
        kind, length = buffer[1:3]
        if kind not in PAYLOAD_FORMATS or length != struct.calcsize(PAYLOAD_FORMATS[kind]):
            del buffer[0]
            continue
        total = 7 + length + 1
        if len(buffer) < total:
            break
        if buffer[total - 1] != FRAME_END:
            del buffer[0]
            continue
        packets.append((kind, bytes(buffer[7:total - 1])))
        del buffer[:total]
    return packets


def select_serial_port(path, vid, pid=-1, *, ports=None):
    """Explicit path wins. Auto-discovery must match exactly one candidate."""
    if path:
        return path
    ports = list_ports.comports() if ports is None else ports
    matches = [p.device for p in ports if p.vid == vid and (pid == -1 or p.pid == pid)]
    if len(matches) > 1:
        raise ValueError(f'Ambiguous serial devices for VID={vid}, PID={pid}: {matches}; set port')
    return matches[0] if matches else None


def parse_nmea(line, *, frame_id, stamp, wall, timeout):
    """Return a real GGA/RMC NavSatFix, source UTC and type; never cache altitude.

    GGA time is matched to the nearest UTC day; RMC supplies its own date.
    Missing/old/malformed data raises ValueError, an explicit no-fix produces
    STATUS_NO_FIX and NaN coordinates. GNSS host UTC must be synchronized.
    """
    data = pynmea2.parse(line, check=True)
    if data.talker not in ('GP', 'GN') or data.sentence_type not in ('GGA', 'RMC'):
        raise ValueError('UNSUPPORTED_SENTENCE')
    utc = data.timestamp
    if utc is None:
        raise ValueError('MISSING_TIME')
    if data.sentence_type == 'RMC':
        day = data.datestamp
        if day is None:
            raise ValueError('MISSING_DATE')
    else:
        day = datetime.fromtimestamp(wall, timezone.utc).date()
    epoch = datetime.combine(day, utc, timezone.utc).timestamp()
    if data.sentence_type == 'GGA':
        if epoch - wall > 43200:
            epoch -= timedelta(days=1).total_seconds()
        elif wall - epoch > 43200:
            epoch += timedelta(days=1).total_seconds()
    if not -0.5 <= wall - epoch <= timeout:
        raise ValueError('STALE')

    msg = NavSatFix()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.status.status = NavSatStatus.STATUS_NO_FIX
    # GN does not identify which constellations actually contributed to the fix.
    msg.status.service = NavSatStatus.SERVICE_GPS if data.talker == 'GP' else 0
    msg.latitude = msg.longitude = msg.altitude = float('nan')
    msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
    if data.sentence_type == 'GGA':
        if data.gps_qual is None:
            raise ValueError('MISSING_FIX_STATUS')
        # Exclude dead reckoning (6), manual (7), simulation (8) and unknown codes.
        valid = data.gps_qual in (1, 2, 3, 4, 5)
    else:
        if data.status not in ('A', 'V'):
            raise ValueError('MISSING_FIX_STATUS')
        # pynmea2 1.15 does not name the optional NMEA 2.3 mode field (index 11).
        mode = data.data[11] if len(data.data) > 11 else ''
        valid = data.status == 'A' and mode in ('', 'A', 'D', 'P', 'F', 'R')
    if valid:
        for raw, direction, digits, allowed in ((data.lat, data.lat_dir, 4, ('N', 'S')),
                                               (data.lon, data.lon_dir, 5, ('E', 'W'))):
            if (direction not in allowed or not re.fullmatch(rf'\d{{{digits}}}(\.\d+)?', raw)
                    or float(raw[digits-2:]) >= 60):
                raise ValueError('INVALID_COORDINATES')
        lat, lon = float(data.latitude), float(data.longitude)
        if not -90 <= lat <= 90 or not -180 <= lon <= 180:
            raise ValueError('INVALID_COORDINATES')
        msg.latitude, msg.longitude = lat, lon
        # Quality 2 can be several differential systems; do not invent SBAS/GBAS.
        msg.status.status = NavSatStatus.STATUS_FIX
        if data.sentence_type == 'GGA' and data.altitude_units == 'M' and data.geo_sep_units == 'M':
            if data.altitude is not None and data.geo_sep:
                altitude = float(data.altitude) + float(data.geo_sep)
                if not math.isfinite(altitude):
                    raise ValueError('INVALID_ALTITUDE')
                msg.altitude = altitude  # NavSatFix needs ellipsoid height, not GGA MSL.
    return msg, epoch, data.sentence_type


class N100HeadingDriver:
    """Original IMU worker and publisher, with freshness and controlled shutdown."""
    def __init__(self, parent_node, port, baud, *, frame_id='imu_link', timeout=2.0):
        self.node = parent_node
        self.frame_id, self.timeout = frame_id, timeout
        self.lock = threading.Lock()
        self.latest = {}
        self.mag = None
        self.source_times = {}
        self.published_source = None
        self.connected = False
        self.stop = threading.Event()
        self.q_rr = (math.cos(PI/2), math.sin(PI/2), 0, 0)
        self.q_z = (math.cos(PI/2), 0, 0, math.sin(PI/2))
        self.pub = self.node.create_publisher(Imu, '/imu/data', 10)
        self.ser = None
        self.thread = None
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1, exclusive=True)
        except (serial.SerialException, OSError) as exc:
            self.node.get_logger().error(f'N100 connection failed: {exc}')
            return
        self.connected = True
        self.thread = threading.Thread(target=self.reader, name='n100_reader', daemon=True)
        self.thread.start()

    def process_packet(self, kind, payload, *, mono, stamp):
        values = struct.unpack(PAYLOAD_FORMATS[kind], payload)
        source_time = values[-1]
        if source_time < 0 or source_time <= self.source_times.get(kind, -1):
            raise ValueError('Repeated/backward N100 timestamp; restart node after sensor restart')
        if kind == TYPE_IMU:
            mag = tuple(v * 1e-7 for v in values[6:9])  # Original scale and indices.
            if not all(math.isfinite(v) for v in mag) or math.hypot(*mag) == 0:
                raise ValueError('Invalid magnetometer vector')
            self.mag = (mag, mono)
        else:
            raw_q = values[6:10]
            norm = math.hypot(*raw_q)
            if not all(math.isfinite(v) for v in raw_q) or norm < 1e-12:
                raise ValueError('Invalid orientation quaternion')
            raw_q = tuple(v / norm for v in raw_q)
            q = quat_mul(self.q_z, quat_mul(self.q_rr, raw_q))
            with self.lock:
                self.latest = dict(q=q, mono=mono, stamp=stamp, source=source_time, heading=None)
        self.source_times[kind] = source_time
        with self.lock:
            # A new magnetometer packet NEVER refreshes the orientation timestamp.
            if self.latest and self.mag is not None:
                mag, mag_time = self.mag
                if mono - mag_time <= self.timeout and mono - self.latest['mono'] <= self.timeout:
                    roll, pitch = quat_to_roll_pitch(self.latest['q'])
                    self.latest['heading'] = tilt_compensated_heading(roll, pitch, *mag)
                else:
                    self.latest['heading'] = None

    def reader(self):
        buffer = bytearray()
        last_byte = time.monotonic()
        try:
            while not self.stop.is_set():
                now = time.monotonic()
                if now - last_byte > self.timeout:
                    buffer.clear()  # A partial packet cannot survive indefinitely.
                chunk = self.ser.read(min(max(self.ser.in_waiting, 1), 4096))
                if not chunk:
                    continue
                last_byte = time.monotonic()
                buffer.extend(chunk)
                packets = extract_packets(buffer)
                if not packets and not buffer:
                    self.node.get_logger().warning('N100 discarded malformed/unsupported frame',
                                                   throttle_duration_sec=5.0)
                for kind, payload in packets:
                    try:
                        self.process_packet(kind, payload, mono=last_byte,
                                            stamp=self.node.get_clock().now().to_msg())
                    except (ValueError, struct.error) as exc:
                        self.node.get_logger().warning(f'N100 rejected packet: {exc}',
                                                       throttle_duration_sec=5.0)
        except (serial.SerialException, OSError) as exc:
            if not self.stop.is_set():
                self.node.get_logger().error(f'N100 disconnected: {exc}; restart node to reconnect')
        finally:
            self.connected = False
            with self.lock:
                self.latest.clear()
            self.ser.close()

    def update_and_publish(self):
        with self.lock:
            data = self.latest.copy()
        if (not self.connected or not data or time.monotonic() - data['mono'] > self.timeout
                or data['source'] == self.published_source):
            return None
        msg = Imu()
        msg.header.stamp = data['stamp']
        msg.header.frame_id = self.frame_id
        qw, qx, qy, qz = data['q']
        msg.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        # All-zero orientation covariance means UNKNOWN, not calibrated certainty.
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0
        self.pub.publish(msg)
        self.published_source = data['source']
        return data['heading']

    def status(self):
        if not self.connected:
            return 'DISCONNECTED'
        with self.lock:
            data = self.latest.copy()
        if not data:
            return 'NO_DATA'
        return 'STALE' if time.monotonic() - data['mono'] > self.timeout else 'OK'

    def close(self):
        self.stop.set()
        if self.ser is not None:
            try:
                self.ser.cancel_read()
            except (serial.SerialException, OSError):
                pass  # Closing an already disconnected port is expected.
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        if self.ser is not None:
            self.ser.close()


class GNSSIMUNode(Node):
    def __init__(self, **kwargs):
        super().__init__('gnss_imu_combined_node', **kwargs)
        defaults = dict(imu_port='', imu_baud=921600, imu_frame_id='imu_link', imu_vid=6790,
                        imu_pid=-1, gnss_port='', gnss_baud=115200, gnss_frame_id='gps_link',
                        gnss_vid=5446, gnss_pid=-1, sensor_timeout_sec=2.0)
        settings = {key: self.declare_parameter(key, value,
                    ParameterDescriptor(read_only=True)).value for key, value in defaults.items()}
        self.timeout = settings['sensor_timeout_sec']
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('sensor_timeout_sec must be finite and positive')
        for name in ('imu', 'gnss'):
            if (settings[f'{name}_baud'] <= 0 or not settings[f'{name}_frame_id']
                    or not 0 <= settings[f'{name}_vid'] <= 65535
                    or not -1 <= settings[f'{name}_pid'] <= 65535):
                raise ValueError(f'Invalid {name} parameters')
        self.imu = None
        self.gps_ser = None
        self.gps_buffer = bytearray()
        self.gps_epochs = {}
        self.gps_last_receive = self.gps_last_byte = None
        self.sensor_status = {}
        self.status_logs = {}
        self.gnss_frame = settings['gnss_frame_id']
        self.nav_sat_pub = self.create_publisher(NavSatFix, '/nav_sat', 10)
        self.get_logger().warning('N100 CRC/header interpretation and mounting are unverified; '
                                  'preserving supplied FC/FD, 56/48-byte layouts and rotations.')
        chosen = {}
        for name in ('imu', 'gnss'):
            try:
                chosen[name] = select_serial_port(settings[f'{name}_port'],
                                                  settings[f'{name}_vid'], settings[f'{name}_pid'])
            except ValueError as exc:
                chosen[name] = None
                self.get_logger().error(str(exc))
            if not chosen[name]:
                self.report_status(name, 'MISSING')
            elif not settings[f'{name}_port'] and settings[f'{name}_pid'] == -1:
                self.get_logger().warning(f'{name}: unique VID={settings[f"{name}_vid"]} match '
                                          'only; verify identity and configure a persistent port')
        if chosen['imu'] and chosen['gnss']:
            # Different symlinks can refer to the same tty; never open it twice.
            from os.path import realpath
            if realpath(chosen['imu']) == realpath(chosen['gnss']):
                self.get_logger().error('IMU and GNSS resolve to the same port; refusing both')
                chosen = dict(imu=None, gnss=None)
                self.report_status('imu', 'CONFIG_ERROR')
                self.report_status('gnss', 'CONFIG_ERROR')
        if chosen['imu']:
            self.imu = N100HeadingDriver(self, chosen['imu'], settings['imu_baud'],
                                         frame_id=settings['imu_frame_id'], timeout=self.timeout)
        if chosen['gnss']:
            try:
                self.gps_ser = serial.Serial(chosen['gnss'], settings['gnss_baud'],
                                             timeout=0, exclusive=True)
                self.report_status('gnss', 'NO_DATA')
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f'GNSS connection failed: {exc}')
                self.report_status('gnss', 'DISCONNECTED')
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.1, self.update_sensors, clock=self.steady_clock)

    def report_status(self, name, status):
        if self.sensor_status.get(name) != status:
            self.sensor_status[name] = status
            key, now = (name, status), time.monotonic()
            if now - self.status_logs.get(key, -math.inf) >= 5.0:
                self.get_logger().info(f'{name.upper()}: {status}')
                self.status_logs[key] = now

    def update_sensors(self):
        if self.imu is not None:
            heading = self.imu.update_and_publish()
            self.report_status('imu', self.imu.status())
            if heading is not None:
                self.get_logger().debug(f'Original magnetic heading: {heading:.2f} degrees')
        if self.gps_ser is not None:
            self.process_gps()
            if (self.gps_ser is not None and self.gps_last_receive is not None
                    and time.monotonic() - self.gps_last_receive > self.timeout):
                self.report_status('gnss', 'STALE')

    def process_gps(self):
        now = time.monotonic()
        if self.gps_last_byte is not None and now - self.gps_last_byte > self.timeout:
            self.gps_buffer.clear()
        try:
            waiting = self.gps_ser.in_waiting
            if waiting > 16384:
                self.gps_ser.reset_input_buffer()
                self.gps_buffer.clear()
                self.report_status('gnss', 'BACKLOG_DROPPED')
                return
            if not waiting:
                return
            self.gps_buffer.extend(self.gps_ser.read(min(waiting, 4096)))
            self.gps_last_byte = now
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'GNSS disconnected: {exc}; restart node to reconnect')
            self.gps_ser.close()
            self.gps_ser = None
            self.gps_buffer.clear()
            self.report_status('gnss', 'DISCONNECTED')
            return
        while b'\n' in self.gps_buffer:
            line, _, rest = self.gps_buffer.partition(b'\n')
            self.gps_buffer[:] = rest
            if not line.startswith((b'$GNGGA', b'$GPGGA', b'$GNRMC', b'$GPRMC')):
                continue
            try:
                if len(line) > 512:
                    raise ValueError('NMEA line too long')
                msg, epoch, kind = parse_nmea(line.decode('ascii').strip(),
                    frame_id=self.gnss_frame, stamp=self.get_clock().now().to_msg(),
                    wall=time.time(), timeout=self.timeout)
                if epoch <= self.gps_epochs.get(kind, -math.inf):
                    raise ValueError('STALE')
                self.gps_epochs[kind] = epoch
                self.gps_last_receive = now
                self.nav_sat_pub.publish(msg)
                self.report_status('gnss', 'OK' if msg.status.status >= 0 else 'NO_FIX')
            except (ValueError, AttributeError, IndexError) as exc:
                self.report_status('gnss', 'STALE' if str(exc) == 'STALE' else 'INVALID_DATA')
                self.get_logger().warning(f'GNSS rejected sentence: {exc}', throttle_duration_sec=5.0)
        if len(self.gps_buffer) > 512:
            self.gps_buffer.clear()
            self.report_status('gnss', 'INVALID_DATA')

    def destroy_node(self):
        if self.imu is not None:
            self.imu.close()
        if self.gps_ser is not None:
            self.gps_ser.close()
            self.gps_ser = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GNSSIMUNode()
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
