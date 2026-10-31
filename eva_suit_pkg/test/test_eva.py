"""Software-only EVA parsing and command validation tests; no hardware is opened."""
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace
import io
import math
import unittest
from unittest.mock import patch

import yaml

try:
    from eva_suit.sensors import (
        FlexGestureLatch, GNSSFixSelector, parse_flex_packet, parse_nmea, select_port,
    )
except ImportError:
    FlexGestureLatch = GNSSFixSelector = parse_flex_packet = parse_nmea = select_port = None

from eva_suit.voice import (
    classify_transcript, normalize_text, parse_whisper_result, read_exact, validate_transcript,
)

try:
    from eva_suit.backend import CommandGate
except ImportError:
    CommandGate = None


WALL = datetime(2026, 10, 31, 12, 0, 0, tzinfo=timezone.utc).timestamp()


def nmea(body):
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return f"${body}*{checksum:02X}"


def gga(quality="1", satellites="14", altitude="10.0", geoid="-3.0"):
    return nmea(f"GNGGA,120000.000,2345.0000,N,09030.0000,E,{quality},{satellites},0.8,"
                f"{altitude},M,{geoid},M,,")


def rmc(status="A", mode="A"):
    return nmea(f"GNRMC,120000.000,{status},2345.0000,N,09030.0000,E,0.0,0.0,"
                f"311026,,,{mode}")


@unittest.skipIf(parse_nmea is None, "pynmea2/pyserial not installed")
class TestGNSS(unittest.TestCase):
    def test_valid_fix_and_ellipsoid_altitude(self):
        fix = parse_nmea(gga(), wall=WALL)
        self.assertTrue(fix["valid"])
        self.assertAlmostEqual(fix["latitude"], 23.75)
        self.assertAlmostEqual(fix["longitude"], 90.5)
        self.assertAlmostEqual(fix["altitude"], 7.0)
        self.assertEqual((fix["satellites"], fix["fix_type"]), (14, "GNSS"))

    def test_no_fix_never_exposes_coordinates(self):
        fix = parse_nmea(gga(quality="0"), wall=WALL)
        self.assertFalse(fix["valid"])
        self.assertTrue(math.isnan(fix["latitude"]))
        self.assertEqual(fix["fix_type"], "NO_FIX")

    def test_rtk_only_from_gga_quality(self):
        self.assertEqual(parse_nmea(gga("4"), wall=WALL)["rtk_status"], "FIXED")
        self.assertEqual(parse_nmea(gga("5"), wall=WALL)["rtk_status"], "FLOAT")
        self.assertEqual(parse_nmea(gga("2"), wall=WALL)["rtk_status"], "NONE")

    def test_bad_checksum_and_stale_time_rejected(self):
        with self.assertRaises(ValueError):
            parse_nmea(gga()[:-2] + "00", wall=WALL)
        with self.assertRaises(ValueError):
            parse_nmea(gga(), wall=WALL + 10)

    def test_port_selection_refuses_ambiguity_and_prefers_filter(self):
        ports = [SimpleNamespace(device="/dev/a", vid=1, pid=2, serial_number="A"),
                 SimpleNamespace(device="/dev/b", vid=1, pid=3, serial_number="B")]
        with patch("eva_suit.sensors.glob.glob", return_value=[]):
            with self.assertRaises(RuntimeError):
                select_port({}, ports=ports)
            self.assertEqual(select_port({"vid": 1, "pid": 3}, ports=ports), "/dev/b")

    def test_mixed_rmc_gga_publishes_only_gga_and_keeps_rtk_metadata(self):
        selector = GNSSFixSelector()
        rmc_fix = parse_nmea(rmc(), wall=WALL)
        gga_fix = parse_nmea(gga(quality="5"), wall=WALL)

        publication, status = selector.process(rmc_fix)
        self.assertIsNone(publication)
        self.assertTrue(status["valid"])

        publication, status = selector.process(gga_fix)
        self.assertIs(publication, gga_fix)
        self.assertEqual((publication["altitude"], status["rtk_status"]), (7.0, "FLOAT"))

        publication, status = selector.process(rmc_fix)
        self.assertIsNone(publication)
        self.assertEqual((status["fix_type"], status["rtk_status"], status["satellites"]),
                         ("RTK_FLOAT", "FLOAT", 14))

    def test_missing_altitude_is_published_as_unknown(self):
        selector = GNSSFixSelector()
        fix = parse_nmea(gga(altitude="", geoid=""), wall=WALL)
        publication, _ = selector.process(fix)
        self.assertIsNotNone(publication)
        self.assertTrue(math.isnan(publication["altitude"]))

    def test_duplicate_gga_epoch_and_rmc_do_not_duplicate_position(self):
        selector = GNSSFixSelector()
        fixes = [parse_nmea(gga(), wall=WALL), parse_nmea(rmc(), wall=WALL),
                 parse_nmea(gga(), wall=WALL)]
        publications = [publication for publication, _ in map(selector.process, fixes)
                        if publication is not None]
        self.assertEqual(len(publications), 1)
        self.assertEqual(publications[0]["sentence"], "GGA")

    def test_invalid_gga_and_rmc_are_not_published(self):
        for fix in (parse_nmea(gga(quality="0"), wall=WALL),
                    parse_nmea(rmc(status="V", mode="N"), wall=WALL)):
            selector = GNSSFixSelector()
            publication, status = selector.process(fix)
            self.assertIsNone(publication)
            self.assertFalse(status["valid"])

        selector = GNSSFixSelector()
        selector.process(parse_nmea(gga(quality="4"), wall=WALL))
        publication, status = selector.process(parse_nmea(rmc(status="V", mode="N"), wall=WALL))
        self.assertIsNone(publication)
        self.assertFalse(status["valid"])
        self.assertEqual(status["fix_type"], "NO_FIX")


@unittest.skipIf(parse_flex_packet is None, "pyserial not installed")
class TestFlex(unittest.TestCase):
    def test_provisional_packet(self):
        packet = parse_flex_packet('{"seq":12,"flex_raw":1830,"timestamp_ms":18000}')
        self.assertEqual(packet, {"seq": 12, "flex_raw": 1830, "timestamp_ms": 18000})

    def test_corrupt_or_invalid_packets(self):
        for text in ("bad", "[]", '{"seq":true,"flex_raw":2}',
                     '{"seq":1,"flex_raw":NaN}', '{"seq":-1,"flex_raw":2}'):
            with self.subTest(text=text), self.assertRaises(ValueError):
                parse_flex_packet(text)

    def test_three_finger_packet(self):
        packet = parse_flex_packet(
            '{"seq":7,"timestamp_ms":1200,"flex_raw":[670,611,750],'
            '"bend_pct":[100,100,100],"gesture":"FOLLOW","active":true,'
            '"event":"FOLLOW_START"}')
        self.assertEqual(packet["format"], "FLEX_JSON_V2")
        self.assertEqual(packet["flex_raw"], [670, 611, 750])
        self.assertEqual(packet["event"], "FOLLOW_START")

    def test_three_finger_packet_validation(self):
        invalid = (
            '{"seq":1,"flex_raw":[1,2],"bend_pct":[1,2],"gesture":"FOLLOW",'
            '"active":false,"event":null}',
            '{"seq":1,"flex_raw":[1,2,3],"bend_pct":[1,2,101],"gesture":"FOLLOW",'
            '"active":false,"event":null}',
            '{"seq":1,"flex_raw":[1,2,3],"bend_pct":[1,2,3],"gesture":"FOLLOW",'
            '"active":false,"event":"FOLLOW_START"}',
            '{"seq":1,"timestamp_ms":-1,"flex_raw":[1,2,3],"bend_pct":[1,2,3],'
            '"gesture":"FOLLOW","active":false,"event":null}',
        )
        for text in invalid:
            with self.subTest(text=text), self.assertRaises(ValueError):
                parse_flex_packet(text)

    def test_follow_latch_emits_edges_only(self):
        latch = FlexGestureLatch()
        start = {"event": "FOLLOW_START"}
        held = {"event": None}
        stop = {"event": "FOLLOW_STOP"}
        self.assertEqual(latch.process(start), "FOLLOW")
        self.assertIsNone(latch.process(start))
        self.assertIsNone(latch.process(held))
        self.assertEqual(latch.process(stop), "STOP")
        self.assertIsNone(latch.process(stop))
        self.assertEqual(latch.process(start), "FOLLOW")
        self.assertEqual(latch.fail_safe(), "STOP")
        self.assertIsNone(latch.fail_safe())


class TestVoice(unittest.TestCase):
    def test_normalization_and_allowed_commands(self):
        self.assertEqual(normalize_text(" Rover, follow! "), "ROVER FOLLOW")
        self.assertEqual(validate_transcript("Rover, follow"), ("FOLLOW", "ACCEPTED_LOCALLY"))
        self.assertEqual(validate_transcript("noise rover please stop"), ("STOP", "ACCEPTED_LOCALLY"))

    def test_wake_unknown_extra_and_ambiguous_rejected(self):
        for phrase, reason in (
            ("follow", "WAKE_PHRASE_MISSING"),
            ("rover dance", "AMBIGUOUS_OR_UNKNOWN_COMMAND"),
            ("rover come follow", "AMBIGUOUS_OR_UNKNOWN_COMMAND"),
            ("rover follow fast", "EXTRA_UNRECOGNIZED_WORDS"),
        ):
            self.assertEqual(validate_transcript(phrase), (None, reason))

    def test_whisper_json_confidence(self):
        phrase, confidence = parse_whisper_result({"transcription": [{
            "text": " Rover follow", "tokens": [
                {"text": " Rover", "p": 0.9}, {"text": " follow", "p": 0.7},
                {"text": "<|endoftext|>", "p": 0.1},
            ]}]})
        self.assertEqual(phrase, "Rover follow")
        self.assertAlmostEqual(confidence, 0.8)

    def test_blank_whisper_json_is_a_safe_rejection(self):
        self.assertEqual(parse_whisper_result({"transcription": []}), ("", None))
        self.assertEqual(parse_whisper_result({"transcription": [{
            "text": " [BLANK_AUDIO]", "tokens": [],
        }]}), ("[BLANK_AUDIO]", None))
        self.assertEqual(classify_transcript("", None, 0.55),
                         (None, "NO_SPEECH_OR_CONFIDENCE"))
        self.assertEqual(classify_transcript("[BLANK_AUDIO]", None, 0.55),
                         (None, "NO_SPEECH_OR_CONFIDENCE"))
        self.assertEqual(classify_transcript("Rover follow", 0.4, 0.55),
                         (None, "LOW_CONFIDENCE"))
        self.assertEqual(classify_transcript("wind howling", 0.9, 0.55),
                         (None, "WAKE_PHRASE_MISSING"))

    def test_partial_microphone_reads_are_reassembled(self):
        class PartialStream:
            def __init__(self):
                self.parts = iter((b"ab", b"c", b"def"))

            def read(self, _size):
                return next(self.parts, b"")

        self.assertEqual(read_exact(PartialStream(), 6), b"abcdef")

    def test_microphone_eof_reports_stderr(self):
        process = SimpleNamespace(poll=lambda: 1, stderr=io.BytesIO(b"USB device disconnected"))
        with self.assertRaisesRegex(RuntimeError, "USB device disconnected"):
            read_exact(io.BytesIO(b"ab"), 4, process)


@unittest.skipIf(CommandGate is None, "ROS 2 Python packages not installed")
class TestCommandGate(unittest.TestCase):
    def test_link_allowlist_and_duplicate_suppression(self):
        gate = CommandGate(["COME", "FOLLOW", "STOP"], 1.5)
        self.assertEqual(gate.validate("follow", "voice", rover_online=False, now=1)[1],
                         "ROVER_DISCONNECTED")
        self.assertTrue(gate.validate("follow", "voice", rover_online=True, now=1)[0])
        self.assertEqual(gate.validate("follow", "voice", rover_online=True, now=2)[1],
                         "DUPLICATE_COOLDOWN")
        self.assertTrue(gate.validate("follow", "voice", rover_online=True, now=3)[0])
        self.assertEqual(gate.validate("fetch", "touch", rover_online=True, now=4)[1],
                         "UNSUPPORTED_COMMAND")


class TestConfiguration(unittest.TestCase):
    def test_default_and_pi_example_configuration(self):
        config_dir = Path(__file__).resolve().parents[1] / "config"
        with (config_dir / "eva.yaml").open(encoding="utf-8") as stream:
            default = yaml.safe_load(stream)
        with (config_dir / "eva-pi.example.yaml").open(encoding="utf-8") as stream:
            pi = yaml.safe_load(stream)
        for config in (default, pi):
            self.assertEqual(set(config), {"serial", "voice", "commands", "ros"})
            self.assertEqual(config["serial"]["gnss"]["baud"], 115200)
            self.assertEqual(config["serial"]["flex"]["baud"], 115200)
            self.assertEqual(config["commands"]["allowed"], ["COME", "FOLLOW", "STOP"])
        self.assertEqual(default["serial"]["gnss"]["port"], "")
        self.assertIn("usb-1a86_USB_Single_Serial", pi["serial"]["gnss"]["port"])
        self.assertIn("usb-Espressif_USB_JTAG", pi["serial"]["flex"]["port"])
        self.assertEqual(pi["voice"]["alsa_device"], "plughw:2,0")
        self.assertTrue(pi["voice"]["model"].endswith("ggml-small.en.bin"))


if __name__ == "__main__":
    unittest.main()
