"""HiPNUC wire-format and ROS conversion tests; no ROS or serial device needed."""
from binascii import crc_hqx
import math
import struct
import unittest

from rover.hipnuc import HI91, HI92, HI81, GRAVITY, decode_imu, extract_packets


# Independent, CRC-bearing captured examples from the supplied vendor parser.
HI91_EXAMPLE = bytes.fromhex(
    '5A A5 4C 00 14 BB 91 08 15 23 09 A2 C4 47 08 15 1C 00 CC E8 61 BE '
    '9A 35 56 3E 65 EA 72 3F 31 D0 7C BD 75 DD C5 BB 6B D7 24 BC 89 88 '
    'FC 40 01 00 6A 41 AB 2A 70 C2 96 D4 50 41 ED 03 43 41 41 F4 F4 C2 '
    'CC CA F8 BE 73 6A 19 BE F0 00 1C 3D 8D 37 5C 3F')
HI92_EXAMPLE = bytes.fromhex(
    '5A A5 30 00 E3 A4 92 23 23 1C 7C 11 61 FF 23 23 00 00 01 00 00 '
    '00 EB FE 7B 00 B6 07 CD 03 70 FC A3 FC 56 1F 00 00 CF 0D 00 00 '
    'C4 07 00 00 F0 26 27 01 C0 02 C2 00')
HI81_EXAMPLE = bytes.fromhex(
    '5A A5 68 00 70 04 81 00 02 01 15 09 43 C8 3E 02 00 00 83 00 3C '
    '00 36 01 8F FB E8 04 31 04 7C 02 B2 F9 BF FE 60 79 00 10 24 18 '
    '07 1C 0A 1B EB 74 9A 0E 8A 0E 2E 18 20 20 3B 10 19 04 65 F1 99 '
    '6A 5D 45 31 61 CE 17 06 D1 00 00 0C 07 01 1C 00 00 00 B9 FC 01 '
    '00 00 02 00 00 00 C6 FF 08 01 E7 FF 7F 12 41 00 4E 61 CE 17 04 '
    'CF 00 00 00 00')


def payload(kind=HI91, stamp=1, q=(1., 0., 0., 0.), yaw=0.,
            acc=(1., 2., 3.), gyr=(10., 20., 30.)):
    if kind != HI91:
        return (HI92_EXAMPLE if kind == HI92 else HI81_EXAMPLE)[6:]
    data = bytearray(76)
    data[0] = HI91
    struct.pack_into('<I', data, 8, stamp)
    struct.pack_into('<3f', data, 12, *acc)
    struct.pack_into('<3f', data, 24, *gyr)
    struct.pack_into('<f', data, 56, yaw)
    struct.pack_into('<4f', data, 60, *q)
    return bytes(data)


def frame(data):
    header = b'\x5a\xa5' + struct.pack('<H', len(data))
    return header + struct.pack('<H', crc_hqx(header + data, 0)) + data


class TestHiPNUC(unittest.TestCase):
    def test_vendor_crc_examples_and_bytewise_reads(self):
        buffer, results = bytearray(), []
        for byte in b'noise\x5a' + HI91_EXAMPLE + HI92_EXAMPLE + HI81_EXAMPLE:
            buffer.append(byte)
            results.extend(extract_packets(buffer))
        self.assertEqual(results, [(HI91, HI91_EXAMPLE[6:]), (HI92, HI92_EXAMPLE[6:]),
                                   (HI81, HI81_EXAMPLE[6:])])
        self.assertEqual(buffer, b'')

    def test_distinct_frames_and_items_do_not_alias(self):
        first, second = payload(stamp=1, yaw=10.), payload(stamp=2, yaw=20.)
        for data in (frame(first) + frame(second), frame(first + second)):
            results = extract_packets(bytearray(data))
            self.assertEqual(results, [(HI91, first), (HI91, second)])
            a, b = (decode_imu(*packet) for packet in results)
            self.assertEqual((a.heading, b.heading), (350., 340.))
            self.assertEqual((a.source_time_ms, b.source_time_ms), (1, 2))

    def test_bad_crc_in_header_or_payload_resynchronizes(self):
        for index in (2, 4, 5, 20, 81):
            damaged = bytearray(HI91_EXAMPLE)
            damaged[index] ^= 1
            results = extract_packets(bytearray(damaged + HI92_EXAMPLE))
            self.assertEqual(results, [(HI92, HI92_EXAMPLE[6:])])

    def test_zero_oversized_unknown_truncated_and_legacy_frames(self):
        for bad in (b'\x5a\xa5\x00\x00\x00\x00', b'\x5a\xa5\xff\xff\x00\x00',
                    frame(b'\xff' + payload()), frame(payload()[:30]),
                    frame(payload() + b'\xff'), b'\xfc\x41\x30' + bytes(52) + b'\xfd'):
            results = extract_packets(bytearray(bad + HI91_EXAMPLE))
            self.assertEqual(results, [(HI91, HI91_EXAMPLE[6:])])

    def test_partial_frame_and_split_sync_retained(self):
        buffer = bytearray(b'noise\x5a')
        self.assertEqual(extract_packets(buffer), [])
        self.assertEqual(buffer, b'\x5a')
        buffer.extend(HI91_EXAMPLE[1:50])
        self.assertEqual(extract_packets(buffer), [])
        buffer.extend(HI91_EXAMPLE[50:])
        self.assertEqual(extract_packets(buffer), [(HI91, HI91_EXAMPLE[6:])])

    def test_pending_buffer_is_bounded(self):
        for data in (b'noise' * 10000, b'\x5a\xa5\x00\x02\x00\x00' + b'x' * 50000):
            buffer = bytearray(data)
            self.assertEqual(extract_packets(buffer), [])
            self.assertLessEqual(len(buffer), 517)

    def test_hi91_vendor_values_and_si_units(self):
        sample = decode_imu(HI91, HI91_EXAMPLE[6:], axes='flu')
        self.assertEqual(sample.source_time_ms, 1840392)
        self.assertAlmostEqual(sample.acceleration[0], -0.220615 * GRAVITY, places=5)
        self.assertAlmostEqual(sample.angular_velocity[0], math.radians(-0.061722), places=7)
        self.assertAlmostEqual(sample.q[0], -0.485922, places=5)
        self.assertAlmostEqual(sample.heading, 212.477, places=3)

    def test_rfu_to_flu_cardinal_headings_and_vectors(self):
        for yaw, heading in ((0., 0.), (-90., 90.), (90., 270.), (180., 180.)):
            half = math.radians(yaw) / 2
            sample = decode_imu(HI91, payload(yaw=yaw, q=(math.cos(half), 0., 0., math.sin(half))))
            w, x, y, z = sample.q
            ros_yaw = math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
            error = (90. - ros_yaw - heading + 180.) % 360. - 180.
            self.assertAlmostEqual(error, 0., places=5)
            self.assertEqual(sample.heading, heading)
            self.assertEqual(sample.acceleration, (2*GRAVITY, -GRAVITY, 3*GRAVITY))
            self.assertEqual(sample.angular_velocity, (math.radians(20), -math.radians(10),
                                                        math.radians(30)))

    def test_tilted_quaternion_changes_body_basis_not_world_basis(self):
        # 90 deg about sensor X: q * q_z differs from q_z * q.
        sample = decode_imu(HI91, payload(q=(math.sqrt(.5), math.sqrt(.5), 0., 0.)))
        for actual, expected in zip(sample.q, (.5, .5, -.5, .5)):
            self.assertAlmostEqual(actual, expected)

    def test_flu_mode_does_not_double_rotate(self):
        sample = decode_imu(HI91, payload(q=(1.01, 0., 0., 0.), yaw=90.), axes='flu')
        self.assertEqual(sample.q, (1., 0., 0., 0.))
        self.assertEqual(sample.heading, 0.)
        self.assertEqual(sample.acceleration, (GRAVITY, 2*GRAVITY, 3*GRAVITY))

    def test_hi92_scaling_and_missing_timestamp(self):
        sample = decode_imu(HI92, HI92_EXAMPLE[6:], axes='flu')
        self.assertIsNone(sample.source_time_ms)
        self.assertEqual(sample.angular_velocity, (0., .001, 0.))
        self.assertAlmostEqual(sample.acceleration[0], -277 * .0048828)
        self.assertAlmostEqual(sample.heading, 90 - 1.988)
        self.assertAlmostEqual(math.hypot(*sample.q), 1.)

    def test_hi81_ins_status_heading_and_reserved_timestamp(self):
        data = bytearray(HI81_EXAMPLE[6:])
        for state in (0, 1, 2, 255):
            data[3] = state
            with self.assertRaisesRegex(ValueError, 'INS attitude'):
                decode_imu(HI81, data)
        for state in (3, 6):
            data[3] = state
            sample = decode_imu(HI81, data)
            self.assertAlmostEqual(sample.heading, 61.9)  # already clockwise
            self.assertIsNone(sample.source_time_ms)
            self.assertEqual(sample.angular_velocity, (.060, -.131, .310))
        with self.assertRaisesRegex(ValueError, 'RFU'):
            decode_imu(HI81, data, axes='flu')

    def test_invalid_orientation_measurements_and_axes_rejected(self):
        for q in ((0., 0., 0., 0.), (2., 0., 0., 0.), (float('nan'), 0., 0., 0.)):
            with self.assertRaises(ValueError):
                decode_imu(HI91, payload(q=q))
        for field in ('yaw', 'acc', 'gyr'):
            for value in (float('nan'), float('inf')):
                kwargs = {field: value if field == 'yaw' else (value, 0., 0.)}
                with self.assertRaises(ValueError):
                    decode_imu(HI91, payload(**kwargs))
        with self.assertRaises(ValueError):
            decode_imu(HI91, payload(), axes='ned')
        for kind, data in ((HI91, b''), (HI91, payload()[:-1]), (HI92, payload()), (0xff, b'\xff')):
            with self.assertRaises(ValueError):
                decode_imu(kind, data)


if __name__ == '__main__':
    unittest.main()
