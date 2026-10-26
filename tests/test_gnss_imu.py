"""Original driver/software tests. All serial enumeration and opens are mocked."""
from datetime import datetime, timezone
import math
import struct
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

try:
    import pynmea2
    import rclpy
    from rclpy.parameter import Parameter
    import serial
    from builtin_interfaces.msg import Time
    from rover.gnss_imu_node import (
        GNSSIMUNode, N100HeadingDriver, TYPE_IMU, TYPE_AHRS, PAYLOAD_FORMATS,
        extract_packets, parse_nmea, quat_mul, quat_to_roll_pitch,
        select_serial_port, tilt_compensated_heading,
    )
except ImportError:
    rclpy = None


NOW = 1792972800.


def payload(kind, stamp=1, q=(1., 0., 0., 0.), mag=(1., 2., 3.)):
    values = [0.] * (12 if kind == TYPE_IMU else 10)
    if kind == TYPE_IMU:
        values[6:9] = mag
    else:
        values[6:10] = q
    return struct.pack(PAYLOAD_FORMATS[kind], *values, stamp)


def frame(kind, stamp=1):
    data = payload(kind, stamp)
    # Opaque header bytes deliberately are NOT claimed as valid CRC values.
    return bytes([0xFC, kind, len(data), 0x01, 0x22, 0x33, 0x44]) + data + b'\xfd'


def sentence(kind='GGA', wall=NOW, talker='GP', **updates):
    instant = datetime.fromtimestamp(wall, timezone.utc)
    utc = instant.strftime('%H%M%S.%f')
    if kind == 'GGA':
        fields = dict(timestamp=utc, lat='0000.0000', lat_dir='N', lon='00000.0000',
                      lon_dir='E', gps_qual='1', num_sats='08', horizontal_dil='1.0',
                      altitude='10.0', altitude_units='M', geo_sep='-3.0', geo_sep_units='M',
                      age_gps_data='', ref_station_id='')
    else:
        fields = dict(timestamp=utc, status='A', lat='0000.0000', lat_dir='N',
                      lon='00000.0000', lon_dir='E', spd_over_grnd='0', true_course='0',
                      datestamp=instant.strftime('%d%m%y'), mag_variation='', mag_var_dir='')
    fields.update(updates)
    cls = pynmea2.GGA if kind == 'GGA' else pynmea2.RMC
    return str(cls(talker, kind, list(fields.values())))


@unittest.skipIf(rclpy is None, 'ROS and serial/NMEA dependencies required')
class TestOriginalProtocol(unittest.TestCase):
    def test_original_packet_layouts_and_chunked_stream(self):
        self.assertEqual(len(payload(TYPE_IMU)), 56)
        self.assertEqual(len(payload(TYPE_AHRS)), 48)
        buffer = bytearray()
        packets = []
        for byte in b'noise' + frame(TYPE_IMU) + frame(TYPE_AHRS):
            buffer.append(byte)
            packets.extend(extract_packets(buffer))
        self.assertEqual(packets, [(TYPE_IMU, payload(TYPE_IMU)), (TYPE_AHRS, payload(TYPE_AHRS))])
        self.assertEqual(buffer, b'')

    def test_corrupt_length_end_type_and_truncation_resynchronize(self):
        good = frame(TYPE_AHRS)
        broken = [frame(TYPE_IMU)[:-1] + b'\x00', b'\xfc\x40\x01\0\0\0\0bad',
                  b'\xfc\x42\x30\0\0\0\0bad', frame(TYPE_IMU)[:20]]
        for item in broken:
            buffer = bytearray(item + good)
            packets = extract_packets(buffer)
            self.assertIn((TYPE_AHRS, payload(TYPE_AHRS)), packets)
        buffer = bytearray(good[:20])
        self.assertEqual(extract_packets(buffer), [])
        buffer.extend(good[20:])
        self.assertEqual(extract_packets(buffer), [(TYPE_AHRS, payload(TYPE_AHRS))])

    def test_quaternion_order_and_original_heading_convention(self):
        self.assertEqual(quat_mul((1, 0, 0, 0), (0, 1, 0, 0)), (0, 1, 0, 0))
        self.assertEqual(quat_mul((0, 0, 0, 1), (0, 1, 0, 0)), (0, 0, 1, 0))
        self.assertEqual(quat_to_roll_pitch((1, 0, 0, 0)), (0., 0.))
        self.assertAlmostEqual(tilt_compensated_heading(0, 0, 1, 0, 0), 0.)
        self.assertAlmostEqual(tilt_compensated_heading(0, 0, 0, 1, 0), 90.)
        self.assertAlmostEqual(tilt_compensated_heading(0, 0, 0, -1, 0), 270.)

    def test_port_selection_is_explicit_or_unique(self):
        ports = [SimpleNamespace(device='/fake/a', vid=6790, pid=123),
                 SimpleNamespace(device='/fake/b', vid=6790, pid=124)]
        with self.assertRaises(ValueError):
            select_serial_port('', 6790, ports=ports)
        self.assertEqual(select_serial_port('', 6790, 124, ports=ports), '/fake/b')
        self.assertEqual(select_serial_port('/chosen', 6790, ports=ports), '/chosen')
        self.assertIsNone(select_serial_port('', 5446, ports=ports))


@unittest.skipIf(rclpy is None, 'ROS and serial/NMEA dependencies required')
class TestN100Driver(unittest.TestCase):
    def setUp(self):
        self.parent = Mock()
        self.parent.create_publisher.return_value = Mock()
        self.serial_patch = patch('rover.gnss_imu_node.serial.Serial')
        self.factory = self.serial_patch.start()
        self.addCleanup(self.serial_patch.stop)
        self.thread_patch = patch('rover.gnss_imu_node.threading.Thread')
        self.thread_patch.start()
        self.addCleanup(self.thread_patch.stop)
        self.mono = 10.
        clock = SimpleNamespace(monotonic=lambda: self.mono)
        self.clock_patch = patch('rover.gnss_imu_node.time', clock)
        self.clock_patch.start()
        self.addCleanup(self.clock_patch.stop)
        self.driver = N100HeadingDriver(self.parent, '/fake/imu', 921600, timeout=.5)
        self.addCleanup(self.driver.close)

    def packet(self, kind=TYPE_AHRS, source=1, **kwargs):
        self.driver.process_packet(kind, payload(kind, source, **kwargs),
                                   mono=self.mono, stamp=Time(sec=123))

    def test_orientation_transform_normalization_and_ros_unknown_fields(self):
        self.packet(q=(2., 0., 0., 0.))
        self.driver.update_and_publish()
        msg = self.driver.pub.publish.call_args.args[0]
        self.assertAlmostEqual(msg.orientation.y, 1.)
        self.assertAlmostEqual(msg.orientation.w, 0.)
        self.assertAlmostEqual(math.hypot(msg.orientation.x, msg.orientation.y,
                                         msg.orientation.z, msg.orientation.w), 1.)
        self.assertEqual(msg.header.frame_id, 'imu_link')
        self.assertEqual(msg.header.stamp.sec, 123)
        self.assertEqual(list(msg.orientation_covariance), [0.] * 9)
        self.assertEqual(msg.angular_velocity_covariance[0], -1.)
        self.assertEqual(msg.linear_acceleration_covariance[0], -1.)

    def test_new_sample_only_not_timer_or_magnetometer_refresh(self):
        self.packet()
        self.driver.update_and_publish()
        for _ in range(4):
            self.mono += .1
            self.packet(TYPE_IMU, source=int(self.mono*100))
            self.driver.update_and_publish()
        self.driver.pub.publish.assert_called_once()
        self.assertEqual(self.driver.latest['mono'], 10.)
        self.assertIsNotNone(self.driver.latest['heading'])
        self.mono += .2
        self.packet(TYPE_IMU, source=9999)
        self.assertIsNone(self.driver.latest['heading'])
        self.assertEqual(self.driver.status(), 'STALE')

    def test_stale_orientation_is_not_published(self):
        self.packet()
        self.mono += .6
        self.driver.update_and_publish()
        self.driver.pub.publish.assert_not_called()

    def test_replay_and_sensor_timestamp_restart_are_rejected(self):
        self.packet(source=10)
        for stamp in (10, 9, -1):
            with self.assertRaises(ValueError):
                self.packet(source=stamp)

    def test_bad_quaternion_and_magnetometer_rejected(self):
        for q in ((0., 0., 0., 0.), (float('nan'), 0., 0., 0.), (float('inf'), 0., 0., 0.)):
            with self.assertRaises(ValueError):
                self.packet(q=q)
        for mag in ((0., 0., 0.), (float('nan'), 1., 2.)):
            with self.assertRaises(ValueError):
                self.packet(TYPE_IMU, mag=mag)
        self.driver.update_and_publish()
        self.driver.pub.publish.assert_not_called()

    def test_serial_failure_clears_latest_and_closes(self):
        self.packet()
        type(self.driver.ser).in_waiting = property(lambda _: (_ for _ in ()).throw(serial.SerialException('gone')))
        self.driver.reader()
        self.assertFalse(self.driver.connected)
        self.assertEqual(self.driver.latest, {})
        self.driver.update_and_publish()
        self.driver.pub.publish.assert_not_called()
        self.driver.ser.close.assert_called()

    def test_shutdown_stops_worker_and_releases_serial(self):
        self.driver.close()
        self.assertTrue(self.driver.stop.is_set())
        self.driver.thread.join.assert_called()
        self.driver.ser.cancel_read.assert_called()
        self.driver.ser.close.assert_called()


@unittest.skipIf(rclpy is None, 'ROS and serial/NMEA dependencies required')
class TestNMEA(unittest.TestCase):
    def parse(self, line, wall=NOW):
        return parse_nmea(line, frame_id='gps_link', stamp=Time(sec=456), wall=wall, timeout=2.)[0]

    def test_gga_equator_prime_meridian_and_ellipsoid_altitude(self):
        msg = self.parse(sentence())
        self.assertEqual((msg.latitude, msg.longitude), (0., 0.))
        self.assertEqual(msg.altitude, 7.)
        self.assertEqual(msg.status.status, 0)
        self.assertEqual(msg.status.service, 1)
        self.assertEqual(msg.header.frame_id, 'gps_link')
        self.assertEqual(msg.position_covariance_type, 0)
        self.assertEqual(list(msg.position_covariance), [0.] * 9)

    def test_rmc_valid_without_gga_does_not_reuse_altitude(self):
        self.parse(sentence(altitude='100'))
        msg = self.parse(sentence('RMC', lat='2330.0000', lon='09030.0000', lat_dir='S', lon_dir='W'))
        self.assertEqual((msg.latitude, msg.longitude), (-23.5, -90.5))
        self.assertEqual(msg.status.status, 0)
        self.assertTrue(math.isnan(msg.altitude))

    def test_no_fix_and_estimated_or_simulated_positions_invalid(self):
        for line in [sentence(gps_qual=q) for q in ('0', '6', '7', '8', '9')] + [sentence('RMC', status='V')]:
            msg = self.parse(line)
            self.assertEqual(msg.status.status, -1)
            self.assertTrue(math.isnan(msg.latitude))
            self.assertTrue(math.isnan(msg.longitude))
            self.assertTrue(math.isnan(msg.altitude))
        msg = self.parse(sentence('RMC', mode='S'))
        self.assertEqual(msg.status.status, -1)

    def test_altitude_missing_geoid_or_units_is_unknown(self):
        for changes in ({'altitude': ''}, {'geo_sep': ''}, {'geo_sep_units': ''}, {'altitude_units': 'F'}):
            self.assertTrue(math.isnan(self.parse(sentence(**changes)).altitude))

    def test_checksum_missing_bad_and_malformed_data_rejected(self):
        valid = sentence()
        for text in (valid.split('*')[0], valid[:-2] + 'XX', 'junk',
                     sentence(lat=''), sentence(lon=''), sentence(lat='0060.000'),
                     sentence(lat='9100.000'), sentence(lat_dir='X'), sentence(timestamp=''),
                     sentence('RMC', datestamp=''), sentence('RMC', status='')):
            with self.subTest(text=text), self.assertRaises(ValueError):
                self.parse(text)

    def test_stale_and_future_sentences_rejected(self):
        for kind in ('GGA', 'RMC'):
            for dt in (-3., 2.):
                with self.assertRaisesRegex(ValueError, 'STALE'):
                    self.parse(sentence(kind, wall=NOW+dt))

    def test_gga_midnight_rollover(self):
        midnight = datetime(2026, 10, 26, tzinfo=timezone.utc).timestamp()
        msg = self.parse(sentence(wall=midnight-.5), wall=midnight+.5)
        self.assertEqual(msg.status.status, 0)

    def test_gn_does_not_invent_constellation_bits(self):
        self.assertEqual(self.parse(sentence(talker='GN')).status.service, 0)


@unittest.skipIf(rclpy is None, 'ROS and serial/NMEA dependencies required')
class TestCombinedNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.port_patch = patch('rover.gnss_imu_node.list_ports.comports', return_value=[])
        self.port_patch.start()
        self.addCleanup(self.port_patch.stop)
        self.serial_patch = patch('rover.gnss_imu_node.serial.Serial',
                                  side_effect=AssertionError('Real serial access forbidden in tests'))
        self.factory = self.serial_patch.start()
        self.addCleanup(self.serial_patch.stop)
        self.node = GNSSIMUNode()
        self.addCleanup(self.node.destroy_node)
        self.wall, self.mono = NOW, 10.
        self.clock_patch = patch('rover.gnss_imu_node.time',
                                SimpleNamespace(time=lambda: self.wall, monotonic=lambda: self.mono))
        self.clock_patch.start()
        self.addCleanup(self.clock_patch.stop)

    def attach(self, data):
        port = Mock()
        port.in_waiting = len(data)
        port.read.return_value = data
        self.node.gps_ser = port
        self.node.nav_sat_pub = Mock()
        return port

    def test_absent_hardware_does_not_open_or_publish(self):
        self.assertEqual(self.node.sensor_status, {'imu': 'MISSING', 'gnss': 'MISSING'})
        self.factory.assert_not_called()
        self.node.nav_sat_pub = Mock()
        self.node.update_sensors()
        self.node.nav_sat_pub.publish.assert_not_called()

    def test_partial_nmea_read_and_replay_do_not_publish_twice(self):
        raw = (sentence() + '\r\n').encode()
        port = self.attach(raw[:20])
        self.node.process_gps()
        self.node.nav_sat_pub.publish.assert_not_called()
        port.read.return_value = raw[20:]
        port.in_waiting = len(raw[20:])
        self.node.process_gps()
        self.node.nav_sat_pub.publish.assert_called_once()
        port.read.return_value = raw
        port.in_waiting = len(raw)
        self.node.process_gps()
        self.node.nav_sat_pub.publish.assert_called_once()

    def test_missing_after_fix_becomes_stale_without_fabricated_publication(self):
        port = self.attach((sentence() + '\n').encode())
        self.node.process_gps()
        self.wall += 3
        self.mono += 3
        port.in_waiting = 0
        self.node.update_sensors()
        self.assertEqual(self.node.sensor_status['gnss'], 'STALE')
        self.node.nav_sat_pub.publish.assert_called_once()

    def test_bad_nmea_cannot_refresh_sensor(self):
        self.attach(b'$GPGGA,bad*00\n')
        self.node.process_gps()
        self.assertEqual(self.node.sensor_status['gnss'], 'INVALID_DATA')
        self.assertIsNone(self.node.gps_last_receive)
        self.node.nav_sat_pub.publish.assert_not_called()

    def test_disconnect_remains_disconnected_not_stale(self):
        port = self.attach(b'bad')
        port.read.side_effect = serial.SerialException('disconnected')
        self.node.gps_last_receive = 0.
        self.node.update_sensors()
        self.assertEqual(self.node.sensor_status['gnss'], 'DISCONNECTED')
        self.assertIsNone(self.node.gps_ser)
        port.close.assert_called_once()

    def test_serial_open_failure_is_contained(self):
        self.factory.side_effect = serial.SerialException('not connected')
        node = GNSSIMUNode(parameter_overrides=[Parameter('gnss_port', value='/fake/gnss'),
                                              Parameter('imu_port', value='/fake/imu')])
        try:
            node.update_sensors()
            self.assertIsNone(node.gps_ser)
            self.assertFalse(node.imu.connected)
        finally:
            node.destroy_node()

    def test_shared_port_refused(self):
        node = GNSSIMUNode(parameter_overrides=[Parameter('gnss_port', value='/fake/same'),
                                              Parameter('imu_port', value='/fake/same')])
        try:
            self.factory.assert_not_called()
            self.assertEqual(node.sensor_status['gnss'], 'CONFIG_ERROR')
        finally:
            node.destroy_node()

    def test_bounded_gnss_backlog_is_discarded(self):
        port = self.attach(b'')
        port.in_waiting = 20000
        self.node.process_gps()
        port.reset_input_buffer.assert_called_once()
        port.read.assert_not_called()
        self.assertEqual(self.node.sensor_status['gnss'], 'BACKLOG_DROPPED')
