"""REAL HiPNUC binary decoding: serial bytes to validated IMU samples; no I/O.

Layouts follow the supplied hipnuc_serial_parser.py (HI91/HI92/HI81).
CRC, WXYZ order and RFU/ENU conventions are documented at
https://download.hipnuc.com/en/products/imu/cum.html and ../ins/cum.html.
"""
from binascii import crc_hqx
from dataclasses import dataclass
import math
import struct
from typing import Optional


HI91, HI92, HI81 = 0x91, 0x92, 0x81
ITEM_SIZES = {HI91: 76, HI92: 48, HI81: 104}
GRAVITY = 9.80665


def extract_packets(buffer):
    """Consume CRC-valid packets, preserving partial reads and split sync bytes.

    Each result is an independent (tag, payload) pair. Unknown/truncated items
    invalidate the entire payload: their bytes must not be scanned as new tags.
    The protocol limit bounds pending data to at most 517 bytes after a call.
    """
    packets = []
    while buffer:
        start = buffer.find(b'\x5a\xa5')
        if start < 0:
            keep = 1 if buffer[-1] == 0x5a else 0
            del buffer[:len(buffer) - keep]
            break
        del buffer[:start]
        if len(buffer) < 6:
            break
        length, received_crc = struct.unpack_from('<HH', buffer, 2)
        if not 1 <= length <= 512:
            del buffer[0]
            continue
        total = length + 6
        if len(buffer) < total:
            break
        packet = bytes(buffer[:total])
        if crc_hqx(packet[:4] + packet[6:], 0) != received_crc:
            del buffer[0]  # A corrupt length may have covered the next good frame.
            continue
        items, offset = [], 6
        while offset < total:
            tag = packet[offset]
            size = ITEM_SIZES.get(tag)
            if size is None or offset + size > total:
                items = []
                break
            items.append((tag, packet[offset:offset + size]))
            offset += size
        packets.extend(items)
        del buffer[:total]
    return packets


def quat_mul(q1, q2):
    """Hamilton product in WXYZ order."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2)


@dataclass(frozen=True)
class IMUSample:
    q: tuple
    angular_velocity: tuple  # rad/s, output body frame
    acceleration: tuple  # m/s^2 including gravity, output body frame
    heading: float  # fused heading; reference depends on sensor operating mode
    source_time_ms: Optional[int]


def decode_imu(kind, payload, *, axes='rfu'):
    """Decode one complete item and convert the sensor body frame to ROS FLU.

    rfu: sensor X right, Y forward, Z up, quaternion maps RFU -> ENU.
    flu: firmware already outputs ROS axes; do not rotate a second time.
    This is an axes conversion, NOT a measured sensor-to-base_link transform.
    """
    if axes not in ('rfu', 'flu'):
        raise ValueError('imu_axes must be rfu or flu')
    if kind not in ITEM_SIZES or len(payload) != ITEM_SIZES[kind] or payload[0] != kind:
        raise ValueError('Invalid HiPNUC item type/length')
    source_time = None
    if kind == HI91:
        source_time = struct.unpack_from('<I', payload, 8)[0]
        acc = tuple(v * GRAVITY for v in struct.unpack_from('<3f', payload, 12))
        gyr = tuple(math.radians(v) for v in struct.unpack_from('<3f', payload, 24))
        yaw = struct.unpack_from('<f', payload, 56)[0]
        q = struct.unpack_from('<4f', payload, 60)
    elif kind == HI92:
        gyr = tuple(v * 0.001 for v in struct.unpack_from('<3h', payload, 10))
        acc = tuple(v * 0.0048828 for v in struct.unpack_from('<3h', payload, 16))
        yaw = struct.unpack_from('<i', payload, 36)[0] * 0.001
        q = tuple(v * 0.0001 for v in struct.unpack_from('<4h', payload, 40))
        # HI92 has no documented monotonic sample counter in the supplied parser.
    else:
        if axes != 'rfu':
            raise ValueError('HI81 requires RFU axes')
        if payload[3] not in (3, 6):
            raise ValueError('HI81 INS attitude invalid or still aligning')
        gyr = tuple(v * 0.001 for v in struct.unpack_from('<3h', payload, 12))
        acc = tuple(v * 0.0048828 for v in struct.unpack_from('<3h', payload, 18))
        yaw = struct.unpack_from('<H', payload, 46)[0] * 0.01
        q = tuple(v * 0.0001 for v in struct.unpack_from('<4h', payload, 48))
        # HI81 offset 90 is reserved in current firmware, not a safe timestamp.
        # GNSS week/TOW may also be unavailable; use receipt freshness instead.
    if not all(math.isfinite(v) for v in (*q, *gyr, *acc, yaw)):
        raise ValueError('Non-finite IMU measurement')
    norm = math.hypot(*q)
    if not 0.5 <= norm <= 1.5:
        raise ValueError('Invalid orientation quaternion norm')
    q = tuple(v / norm for v in q)
    if axes == 'rfu':
        # v_FLU = (y_RFU, -x_RFU, z_RFU).
        # q_FLU->ENU = q_RFU->ENU * q_FLU->RFU (RIGHT multiplication).
        root_half = math.sqrt(0.5)
        q = quat_mul(q, (root_half, 0., 0., root_half))
        acc = (acc[1], -acc[0], acc[2])
        gyr = (gyr[1], -gyr[0], gyr[2])
    # Unlike HI91/HI92, HI81 already reports clockwise compass heading.
    heading = (yaw if kind == HI81 else (0. if axes == 'rfu' else 90.) - yaw) % 360.
    return IMUSample(q, gyr, acc, heading, source_time)
