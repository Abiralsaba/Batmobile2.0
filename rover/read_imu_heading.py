#!/usr/bin/env python3
"""Read HiPNUC IMU/GNSS frames and display orientation plus heading.

Run from the extracted example-project directory:
    python3 read_imu_heading.py --port /dev/ttyUSB0 --baudrate 115200
"""

import argparse
import logging
import math
import sys
import time

import serial
from parsers.hipnuc_serial_parser import hipnuc_parser


logging.getLogger().setLevel(logging.ERROR)


def heading_from_yaw(yaw_deg: float) -> float:
    """Convert sensor yaw to a 0..360 degree heading."""
    return (-yaw_deg) % 360.0


def compass_direction(heading: float) -> str:
    """Return the nearest 8-point compass direction."""
    directions = ("N", "NE", "E", "SE", "S", "SW", "W", "NW")
    return directions[int((heading + 22.5) // 45) % 8]


def fmt_vector(values, precision=3):
    if values is None:
        return "N/A"
    return "X={:.{p}f}, Y={:.{p}f}, Z={:.{p}f}".format(
        values[0], values[1], values[2], p=precision
    )


def display_frame(frame, frame_rate):
    print("\033[2J\033[H", end="")  # Clear terminal and move cursor to top
    print("=" * 66)
    print("              HiPNUC IMU / GNSS LIVE MONITOR")
    print("=" * 66)
    print("Frame type       : HI{:02X}".format(getattr(frame, "frame_type", 0)))
    print("Frame rate       : {} frames/s".format(frame_rate))
    print("Temperature      : {} C".format(
        "{:.1f}".format(frame.temperature) if frame.temperature is not None else "N/A"
    ))

    print("\n--- ORIENTATION ---")
    print("Roll             : {} deg".format(
        "{:.3f}".format(frame.roll) if frame.roll is not None else "N/A"
    ))
    print("Pitch            : {} deg".format(
        "{:.3f}".format(frame.pitch) if frame.pitch is not None else "N/A"
    ))
    print("Yaw (sensor)     : {} deg".format(
        "{:.3f}".format(frame.yaw) if frame.yaw is not None else "N/A"
    ))

    if frame.yaw is not None:
        heading = heading_from_yaw(frame.yaw)
        print("\n--- HEADING ---")
        print("Heading          : {:8.3f} deg".format(heading))
        print("Compass direction: {}".format(compass_direction(heading)))
        print("Note             : Normalized from the sensor's fused yaw")
    else:
        print("\nHeading          : Waiting for yaw data...")

    print("\n--- IMU MEASUREMENTS ---")
    print("Accelerometer    : {} G".format(fmt_vector(frame.acc)))
    print("Gyroscope        : {} deg/s".format(fmt_vector(frame.gyr)))
    print("Magnetometer     : {} uT".format(fmt_vector(frame.mag)))

    if frame.quat is not None:
        print("Quaternion       : {}".format(
            ", ".join("{:.5f}".format(v) for v in frame.quat)
        ))

    if frame.ins_lat is not None and frame.ins_lon is not None:
        print("\n--- GNSS / INS ---")
        print("Latitude         : {:.7f} deg".format(frame.ins_lat))
        print("Longitude        : {:.7f} deg".format(frame.ins_lon))
        if frame.ins_msl is not None:
            print("Altitude MSL     : {:.3f} m".format(frame.ins_msl))
        if frame.vel_enu is not None:
            print("Velocity ENU     : {} m/s".format(fmt_vector(frame.vel_enu)))
        if frame.solq_heading is not None:
            print("Heading quality  : {}".format(frame.solq_heading))

    print("\nPress Ctrl+C to stop.")
    sys.stdout.flush()


def main():
    arg_parser = argparse.ArgumentParser(
        description="Read HiPNUC IMU data and display calculated heading."
    )
    arg_parser.add_argument("-p", "--port", required=True,
                            help="Serial port, e.g. /dev/ttyUSB0 or COM3")
    arg_parser.add_argument("-b", "--baudrate", type=int, default=115200,
                            help="Serial baud rate (default: 115200)")
    args = arg_parser.parse_args()

    if args.baudrate <= 0:
        arg_parser.error("baudrate must be a positive integer")

    parser = hipnuc_parser()
    frame_count = 0
    displayed_rate = 0
    rate_start = time.monotonic()
    last_display = 0.0
    latest_frame = None

    try:
        print("Connecting to {} at {} baud...".format(args.port, args.baudrate))
        with serial.Serial(args.port, args.baudrate, timeout=0.1) as ser:
            print("Connected. Waiting for valid HiPNUC frames...")
            while True:
                data = ser.read(ser.in_waiting or 1)
                if not data:
                    continue

                frames = parser.parse(data)
                if frames:
                    latest_frame = frames[-1]
                    frame_count += len(frames)

                now = time.monotonic()
                if now - rate_start >= 1.0:
                    displayed_rate = frame_count
                    frame_count = 0
                    rate_start = now

                if latest_frame is not None and now - last_display >= 0.2:
                    display_frame(latest_frame, displayed_rate)
                    last_display = now

    except KeyboardInterrupt:
        print("\nStopped by user.")
    except (serial.SerialException, PermissionError) as exc:
        print("Serial connection error: {}".format(exc), file=sys.stderr)
        print("On Ubuntu, check the port and permissions (e.g. dialout group).",
              file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
