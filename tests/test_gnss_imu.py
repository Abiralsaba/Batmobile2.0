"""ROS wrapper/software tests. All serial enumeration and opens are mocked."""
from datetime import datetime, timezone
import math
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from rover.hipnuc import HI91, HI92, GRAVITY
from test_hipnuc import payload, frame

try:
    import pynmea2
    import rclpy
    from rclpy.parameter import Parameter
    import serial
    from builtin_interfaces.msg import Time
except ImportError:
    rclpy = None
else:
    from rover.gnss_imu_node import (
        GNSSIMUNode, N100HeadingDriver, parse_nmea, select_serial_port,
    )


NOW = 1792972800.


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
class TestPortSelection(unittest.TestCase):
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
        self.driver = N100HeadingDriver(self.parent, '/fake/imu', 115200, timeout=.5)
        self.addCleanup(self.driver.close)

    def packet(self, kind=HI91, source=1, **kwargs):
        self.driver.process_packet(kind, payload(kind, source, **kwargs),
                                   mono=self.mono, stamp=Time(sec=123))

    def test_orientation_transform_normalization_and_ros_unknown_fields(self):
        self.packet(q=(1.01, 0., 0., 0.))
        self.driver.update_and_publish()
        msg = self.driver.pub.publish.call_args.args[0]
        self.assertAlmostEqual(msg.orientation.z, math.sqrt(.5))
        self.assertAlmostEqual(msg.orientation.w, math.sqrt(.5))
        self.assertAlmostEqual(math.hypot(msg.orientation.x, msg.orientation.y,
                                         msg.orientation.z, msg.orientation.w), 1.)
        self.assertEqual(msg.header.frame_id, 'imu_link')
        self.assertEqual(msg.header.stamp.sec, 123)
        self.assertEqual(list(msg.orientation_covariance), [0.] * 9)
        self.assertEqual(list(msg.angular_velocity_covariance), [0.] * 9)
        self.assertEqual(list(msg.linear_acceleration_covariance), [0.] * 9)
        self.assertAlmostEqual(msg.angular_velocity.x, math.radians(20))
        self.assertAlmostEqual(msg.angular_velocity.y, -math.radians(10))
        self.assertAlmostEqual(msg.linear_acceleration.x, 2*GRAVITY)
        self.assertAlmostEqual(msg.linear_acceleration.z, 3*GRAVITY)

    def test_new_sample_only_not_timer_refresh(self):
        self.packet()
        self.driver.update_and_publish()
        for _ in range(4):
            self.mono += .1
            self.driver.update_and_publish()
        self.driver.pub.publish.assert_called_once()
        self.assertEqual(self.driver.latest['mono'], 10.)
        self.mono += .2
        self.assertEqual(self.driver.status(), 'STALE')
        self.packet(source=2)
        self.driver.update_and_publish()
        self.assertEqual(self.driver.pub.publish.call_count, 2)
        self.assertEqual(self.driver.status(), 'OK')

    def test_hi92_no_timestamp_still_publishes_once_per_received_sample(self):
        self.packet(HI92)
        self.driver.update_and_publish()
        self.driver.update_and_publish()
        self.driver.pub.publish.assert_called_once()
        self.packet(HI92)
        self.driver.update_and_publish()
        self.assertEqual(self.driver.pub.publish.call_count, 2)

    def test_stale_orientation_is_not_published(self):
        self.packet()
        self.mono += .6
        self.driver.update_and_publish()
        self.driver.pub.publish.assert_not_called()

    def test_replay_and_sensor_timestamp_restart_are_rejected(self):
        self.packet(source=10)
        for stamp in (10, 9, 0):
            with self.assertRaises(ValueError):
                self.packet(source=stamp)
        self.assertEqual(self.driver.latest['mono'], 10.)

    def test_bad_quaternion_and_measurements_rejected(self):
        for q in ((0., 0., 0., 0.), (float('nan'), 0., 0., 0.), (float('inf'), 0., 0., 0.)):
            with self.assertRaises(ValueError):
                self.packet(q=q)
        for acc in ((float('inf'), 0., 0.), (float('nan'), 1., 2.)):
            with self.assertRaises(ValueError):
                self.packet(acc=acc)
        self.driver.update_and_publish()
        self.driver.pub.publish.assert_not_called()

    def test_reader_accepts_latest_of_multiple_frames_and_ignores_bad_crc(self):
        good = frame(payload(stamp=1, yaw=20.)) + frame(payload(stamp=2, yaw=40.))
        bad = bytearray(frame(payload(stamp=3, yaw=60.)))
        bad[-1] ^= 1
        self.driver.ser.in_waiting = len(good + bad)

        def read(_):
            self.driver.stop.set()  # Finish after processing this chunk.
            return good + bad

        self.driver.ser.read.side_effect = read
        received = []
        process = self.driver.process_packet

        def capture(*args, **kwargs):
            process(*args, **kwargs)
            received.append(self.driver.latest['sample'].heading)

        with patch.object(self.driver, 'process_packet', side_effect=capture):
            self.driver.reader()
        self.assertEqual(received, [340., 320.])
        self.assertEqual(self.driver.latest, {})  # Worker exit invalidates freshness.

    def test_backlog_is_dropped_without_publishing_buffered_data(self):
        self.packet()
        self.driver.ser.in_waiting = 20000
        self.driver.ser.reset_input_buffer.side_effect = self.driver.stop.set
        self.driver.reader()
        self.driver.ser.reset_input_buffer.assert_called_once()
        self.driver.ser.read.assert_not_called()
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
