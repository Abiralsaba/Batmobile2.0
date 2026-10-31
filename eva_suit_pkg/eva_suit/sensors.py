"""Independent reconnecting readers for Beitian NMEA GNSS and ESP32 flex JSON."""
from datetime import datetime, timedelta, timezone
import glob
import json
import math
import os
import re
import threading
import time

import pynmea2
import serial
from serial.tools import list_ports


def _stable_device(device):
    """Return a matching /dev/serial/by-id symlink when Linux exposes one."""
    real = os.path.realpath(device)
    for candidate in sorted(glob.glob("/dev/serial/by-id/*")):
        if os.path.realpath(candidate) == real:
            return candidate
    return device


def select_port(config, *, ports=None, excluded=()):
    """Select an explicit port or one unique USB match; ambiguity is an error."""
    explicit = str(config.get("port") or "").strip()
    if explicit:
        if not os.path.exists(explicit):
            raise FileNotFoundError(explicit)
        return explicit
    ports = list(list_ports.comports() if ports is None else ports)
    excluded = {os.path.realpath(path) for path in excluded if path}
    vid, pid = config.get("vid"), config.get("pid")
    serial_number = str(config.get("serial_number") or "").strip()
    matches = []
    for item in ports:
        if os.path.realpath(item.device) in excluded:
            continue
        if vid is not None and item.vid != int(vid):
            continue
        if pid is not None and item.pid != int(pid):
            continue
        if serial_number and item.serial_number != serial_number:
            continue
        matches.append(_stable_device(item.device))
    if len(matches) != 1:
        detail = ", ".join(matches) if matches else "none"
        raise RuntimeError(f"expected one matching serial device, found {detail}; configure port/USB IDs")
    return matches[0]


def _source_epoch(data, wall):
    stamp = data.timestamp
    if stamp is None:
        raise ValueError("MISSING_TIME")
    if data.sentence_type == "RMC":
        if data.datestamp is None:
            raise ValueError("MISSING_DATE")
        day = data.datestamp
    else:
        day = datetime.fromtimestamp(wall, timezone.utc).date()
    epoch = datetime.combine(day, stamp, timezone.utc).timestamp()
    if data.sentence_type == "GGA":
        if epoch - wall > 43200:
            epoch -= timedelta(days=1).total_seconds()
        elif wall - epoch > 43200:
            epoch += timedelta(days=1).total_seconds()
    return epoch


def parse_nmea(line, *, wall=None, max_age_sec=3.0):
    """Parse checksum-valid GGA/RMC into a transport-neutral fix dictionary."""
    if len(line) > 512:
        raise ValueError("LINE_TOO_LONG")
    data = pynmea2.parse(line, check=True)
    if data.talker not in ("GP", "GN") or data.sentence_type not in ("GGA", "RMC"):
        raise ValueError("UNSUPPORTED_SENTENCE")
    wall = time.time() if wall is None else wall
    epoch = _source_epoch(data, wall)
    age = wall - epoch
    if not -0.5 <= age <= max_age_sec:
        raise ValueError("STALE_SOURCE_TIME")

    if data.sentence_type == "GGA":
        quality = int(data.gps_qual or 0)
        valid = quality in (1, 2, 3, 4, 5)
    else:
        mode = data.data[11] if len(data.data) > 11 else ""
        valid = data.status == "A" and mode in ("", "A", "D", "P", "F", "R")
        quality = None

    latitude = longitude = altitude = math.nan
    if valid:
        for raw, direction, digits, allowed in (
            (data.lat, data.lat_dir, 4, ("N", "S")),
            (data.lon, data.lon_dir, 5, ("E", "W")),
        ):
            if (direction not in allowed or not re.fullmatch(rf"\d{{{digits}}}(\.\d+)?", raw)
                    or float(raw[digits - 2:]) >= 60):
                raise ValueError("INVALID_COORDINATES")
        latitude, longitude = float(data.latitude), float(data.longitude)
        if not -90 <= latitude <= 90 or not -180 <= longitude <= 180:
            raise ValueError("INVALID_COORDINATES")
        if (data.sentence_type == "GGA" and data.altitude_units == "M"
                and data.geo_sep not in (None, "") and data.geo_sep_units == "M"
                and data.altitude is not None):
            altitude = float(data.altitude) + float(data.geo_sep)
            if not math.isfinite(altitude):
                raise ValueError("INVALID_ALTITUDE")

    fix_type = {1: "GNSS", 2: "DGNSS", 3: "PPS", 4: "RTK_FIXED", 5: "RTK_FLOAT"}.get(
        quality, "GNSS" if valid else "NO_FIX")
    rtk = {4: "FIXED", 5: "FLOAT"}.get(quality, "NONE" if quality is not None else "UNKNOWN")
    satellites = None
    if data.sentence_type == "GGA" and data.num_sats not in (None, ""):
        satellites = int(data.num_sats)
        if not 0 <= satellites <= 200:
            raise ValueError("INVALID_SATELLITE_COUNT")
    return {
        "valid": valid,
        "latitude": latitude,
        "longitude": longitude,
        "altitude": altitude,
        "satellites": satellites,
        "fix_type": fix_type,
        "rtk_status": rtk,
        "quality": quality,
        "source_epoch": epoch,
        "source_age_sec": max(0.0, age),
        "sentence": data.sentence_type,
        "talker": data.talker,
    }


class GNSSFixSelector:
    """Publish each valid GGA epoch once; retain same-epoch GGA quality for RMC status."""
    def __init__(self):
        self.last_published_epoch = None
        self.last_gga = None

    def process(self, fix):
        if fix["sentence"] == "GGA":
            self.last_gga = fix
            epoch = fix["source_epoch"]
            if (fix["valid"] and (self.last_published_epoch is None
                                  or epoch > self.last_published_epoch + 1e-6)):
                self.last_published_epoch = epoch
                return fix, fix
            return None, fix

        status = fix
        if (fix["valid"] and self.last_gga is not None and self.last_gga["valid"]
                and math.isclose(fix["source_epoch"], self.last_gga["source_epoch"],
                                 rel_tol=0.0, abs_tol=1e-3)):
            status = dict(fix)
            for field in ("satellites", "fix_type", "rtk_status", "quality"):
                status[field] = self.last_gga[field]
        return None, status


def parse_flex_packet(line):
    """Validate legacy single-sensor or three-finger newline-delimited JSON."""
    if len(line) > 1024:
        raise ValueError("PACKET_TOO_LONG")
    try:
        packet = json.loads(line)
    except json.JSONDecodeError as exc:
        raise ValueError("INVALID_JSON") from exc
    if not isinstance(packet, dict) or type(packet.get("seq")) is not int or packet["seq"] < 0:
        raise ValueError("INVALID_SEQUENCE")
    if "timestamp_ms" in packet and (type(packet["timestamp_ms"]) is not int
                                      or packet["timestamp_ms"] < 0):
        raise ValueError("INVALID_DEVICE_TIMESTAMP")
    values = packet.get("flex_raw")
    if isinstance(values, list):
        if (len(values) != 3 or any(isinstance(value, bool)
                or not isinstance(value, (int, float)) or not math.isfinite(value)
                for value in values)):
            raise ValueError("INVALID_FLEX_VALUES")
        percentages = packet.get("bend_pct")
        if (not isinstance(percentages, list) or len(percentages) != 3
                or any(isinstance(value, bool) or not isinstance(value, (int, float))
                       or not math.isfinite(value) or not 0 <= value <= 100
                       for value in percentages)):
            raise ValueError("INVALID_BEND_PERCENTAGES")
        if packet.get("gesture") != "FOLLOW" or type(packet.get("active")) is not bool:
            raise ValueError("INVALID_FLEX_GESTURE")
        event = packet.get("event")
        if event not in (None, "FOLLOW_START", "FOLLOW_STOP"):
            raise ValueError("INVALID_FLEX_EVENT")
        if ((event == "FOLLOW_START" and not packet["active"])
                or (event == "FOLLOW_STOP" and packet["active"])):
            raise ValueError("INCONSISTENT_FLEX_EVENT")
        result = {key: packet[key] for key in (
            "seq", "flex_raw", "bend_pct", "timestamp_ms", "gesture", "active", "event")
            if key in packet}
        result["format"] = "FLEX_JSON_V2"
        return result
    if (isinstance(values, bool) or not isinstance(values, (int, float))
            or not math.isfinite(values)):
        raise ValueError("INVALID_FLEX_VALUE")
    return {key: packet[key] for key in ("seq", "flex_raw", "timestamp_ms") if key in packet}


class FlexGestureLatch:
    """Convert validated edge events into one FOLLOW and one STOP command."""
    def __init__(self):
        self.active = False

    def process(self, packet):
        event = packet.get("event")
        if event == "FOLLOW_START" and not self.active:
            self.active = True
            return "FOLLOW"
        if event == "FOLLOW_STOP" and self.active:
            self.active = False
            return "STOP"
        return None

    def fail_safe(self):
        if self.active:
            self.active = False
            return "STOP"
        return None


class _SerialWorker:
    def __init__(self, name, config, common, on_data, on_status, *, excluded=()):
        self.name, self.config, self.common = name, config, common
        self.on_data, self.on_status = on_data, on_status
        self.excluded = excluded
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, name=f"eva_{name}", daemon=True)
        self.serial = None
        self._last_status = None

    def start(self):
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.serial is not None:
            try:
                self.serial.cancel_read()
            except (AttributeError, OSError, serial.SerialException):
                pass
        self.thread.join(timeout=2.0)
        self._close()

    def _status(self, state, **fields):
        payload = {"connected": state not in ("DISCONNECTED", "CONFIG_ERROR"), "state": state}
        payload.update(fields)
        marker = json.dumps(payload, sort_keys=True, allow_nan=False)
        if marker != self._last_status:
            self._last_status = marker
            self.on_status(payload)

    def _close(self):
        if self.serial is not None:
            try:
                self.serial.close()
            except (OSError, serial.SerialException):
                pass
            self.serial = None

    def _open(self, port):
        try:
            return serial.Serial(port, int(self.config["baud"]), timeout=0.2, exclusive=True)
        except TypeError:  # Older pyserial platform adapter.
            return serial.Serial(port, int(self.config["baud"]), timeout=0.2)

    def _run(self):
        delay = float(self.common.get("reconnect_sec", 2.0))
        while not self.stop_event.is_set():
            try:
                port = select_port(self.config, excluded=self.excluded)
                self.serial = self._open(port)
                self._last_status = None
                self._status("NO_DATA", port=port, baud=int(self.config["baud"]))
                self.read_loop(port)
            except (FileNotFoundError, RuntimeError, OSError, serial.SerialException) as exc:
                self._status("DISCONNECTED", error=str(exc))
            finally:
                self._close()
            self.stop_event.wait(delay)

    def read_loop(self, port):
        raise NotImplementedError


class GNSSReader(_SerialWorker):
    def __init__(self, config, common, on_data, on_status, *, excluded=()):
        super().__init__("gnss", config, common, on_data, on_status, excluded=excluded)
        self.selector = GNSSFixSelector()

    def read_loop(self, port):
        last_valid = time.monotonic()
        stale = float(self.common.get("stale_sec", 2.0))
        while not self.stop_event.is_set():
            raw = self.serial.readline(513)
            if not raw:
                if time.monotonic() - last_valid > stale:
                    self._status("STALE", port=port, fix_valid=False, data_age_sec=round(
                        time.monotonic() - last_valid, 2), rtk_status="UNKNOWN")
                continue
            if not raw.startswith((b"$GNGGA", b"$GPGGA", b"$GNRMC", b"$GPRMC")):
                continue
            try:
                fix = parse_nmea(raw.decode("ascii").strip(), max_age_sec=max(3.0, stale))
            except (UnicodeError, ValueError, AttributeError, IndexError) as exc:
                self._status("INVALID_DATA", port=port, fix_valid=False, error=str(exc),
                             rtk_status="UNKNOWN")
                continue
            last_valid = time.monotonic()
            publication, status = self.selector.process(fix)
            if publication is not None:
                self.on_data(publication)
            self._status("FIX" if status["valid"] else "NO_FIX", port=port,
                         fix_valid=status["valid"], fix_type=status["fix_type"],
                         satellites=status["satellites"], data_age_sec=fix["source_age_sec"],
                         rtk_status=status["rtk_status"], source_sentence=fix["sentence"],
                         source_epoch=fix["source_epoch"],
                         position_published=publication is not None)


class FlexReader(_SerialWorker):
    def __init__(self, config, common, on_data, on_status, *, excluded=()):
        super().__init__("flex", config, common, on_data, on_status, excluded=excluded)

    def read_loop(self, port):
        last_data, last_seq = time.monotonic(), None
        stale = float(self.common.get("stale_sec", 2.0))
        while not self.stop_event.is_set():
            raw = self.serial.readline(1025)
            if not raw:
                if time.monotonic() - last_data > stale:
                    self._status("STALE", port=port, data_age_sec=round(
                        time.monotonic() - last_data, 2))
                continue
            try:
                packet = parse_flex_packet(raw.decode("utf-8").strip())
                if last_seq is not None and packet["seq"] <= last_seq:
                    raise ValueError("DUPLICATE_OR_REVERSED_SEQUENCE")
            except (UnicodeError, ValueError) as exc:
                self._status("INVALID_DATA", port=port, error=str(exc))
                continue
            last_seq, last_data = packet["seq"], time.monotonic()
            self.on_data(packet)
            self._status("OK", port=port, last_seq=last_seq, data_age_sec=0.0)
