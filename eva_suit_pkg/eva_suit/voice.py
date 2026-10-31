"""Offline ALSA capture, lightweight energy VAD and strict Whisper command filtering."""
from collections import deque
import audioop
import json
import os
import re
import signal
import subprocess
import tempfile
import threading
import time
import wave


COMMANDS = ("COME", "FOLLOW", "STOP")


def normalize_text(text):
    return " ".join(re.findall(r"[A-Z]+", text.upper()))


def validate_transcript(text, wake_phrase="ROVER"):
    """Require a wake phrase followed by exactly one restricted command."""
    words = normalize_text(text).split()
    wake = normalize_text(wake_phrase).split()
    positions = [i for i in range(len(words)) if words[i:i + len(wake)] == wake]
    if not positions:
        return None, "WAKE_PHRASE_MISSING"
    tail = words[positions[-1] + len(wake):]
    found = [word for word in tail if word in COMMANDS]
    if len(found) != 1:
        return None, "AMBIGUOUS_OR_UNKNOWN_COMMAND"
    if any(word not in COMMANDS and word not in ("PLEASE",) for word in tail):
        return None, "EXTRA_UNRECOGNIZED_WORDS"
    return found[0], "ACCEPTED_LOCALLY"


def discover_microphone(configured=""):
    if configured:
        return configured
    result = subprocess.run(["arecord", "-l"], text=True, capture_output=True, timeout=3,
                            check=False)
    candidates = []
    for line in result.stdout.splitlines():
        match = re.search(r"card\s+\d+:\s*([^\s]+).*device\s+(\d+):", line, re.I)
        if match and "usb" in line.lower():
            candidates.append(f"plughw:CARD={match.group(1)},DEV={match.group(2)}")
    candidates = list(dict.fromkeys(candidates))
    if len(candidates) != 1:
        raise RuntimeError(f"expected one USB capture device, found {candidates or 'none'}")
    return candidates[0]


def parse_whisper_result(data):
    """Extract text and mean lexical-token probability from whisper.cpp full JSON."""
    segments = data.get("transcription")
    if not isinstance(segments, list):
        raise ValueError("Whisper JSON has no transcription array")
    text, probabilities = [], []
    for segment in segments:
        if not isinstance(segment, dict):
            continue
        text.append(str(segment.get("text", "")).strip())
        for token in segment.get("tokens", []):
            token_text = str(token.get("text", ""))
            probability = token.get("p")
            if (not token_text.startswith("<|") and re.search(r"[A-Za-z]", token_text)
                    and isinstance(probability, (int, float))):
                probabilities.append(float(probability))
    phrase = " ".join(part for part in text if part)
    confidence = sum(probabilities) / len(probabilities) if probabilities else None
    return phrase, confidence


def classify_transcript(phrase, confidence, threshold, wake_phrase="ROVER"):
    """Apply confidence and exact wake/command checks without guessing missing scores."""
    if not 0.0 <= threshold <= 1.0:
        raise ValueError("min_mean_token_confidence must be in [0, 1]")
    if not phrase or confidence is None:
        return None, "NO_SPEECH_OR_CONFIDENCE"
    if confidence < threshold:
        return None, "LOW_CONFIDENCE"
    return validate_transcript(phrase, wake_phrase)


def read_exact(stream, size, process=None):
    """Assemble one PCM frame from partial buffered reads or raise on real EOF."""
    chunks = []
    remaining = size
    while remaining:
        chunk = stream.read(remaining)
        if not chunk:
            detail = ""
            if process is not None and process.poll() is not None and process.stderr is not None:
                detail = process.stderr.read().decode(errors="replace").strip()[-300:]
            raise RuntimeError("microphone stream ended" + (f": {detail}" if detail else ""))
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


class VoiceWorker:
    """Capture process and inference worker; never writes persistent recordings."""
    def __init__(self, config, on_event, on_status):
        self.config, self.on_event, self.on_status = config, on_event, on_status
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, name="eva_voice", daemon=True)
        self.process = None
        self._last_status = None

    def _status(self, state, **fields):
        marker = (state, tuple(sorted(fields.items())))
        if marker != self._last_status:
            self._last_status = marker
            self.on_status({"state": state, **fields})

    def start(self):
        if self.config.get("enabled", True):
            self.thread.start()
        else:
            self._status("DISABLED", microphone_connected=False, whisper_loaded=False)

    def stop(self):
        self.stop_event.set()
        if self.process is not None and self.process.poll() is None:
            self.process.send_signal(signal.SIGTERM)
        if self.thread.is_alive():
            self.thread.join(timeout=3.0)

    def _write_wav(self, pcm, sample_rate):
        handle = tempfile.NamedTemporaryFile(prefix="eva_voice_", suffix=".wav", delete=False)
        handle.close()
        with wave.open(handle.name, "wb") as output:
            output.setnchannels(1)
            output.setsampwidth(2)
            output.setframerate(sample_rate)
            output.writeframes(pcm)
        return handle.name

    def _transcribe(self, pcm, sample_rate):
        path = self._write_wav(pcm, sample_rate)
        output_base = path[:-4] + "_result"
        output_json = output_base + ".json"
        try:
            result = subprocess.run([
                self.config["whisper_binary"], "-m", self.config["model"], "-f", path,
                "-l", "en", "-ojf", "-of", output_base, "-np",
            ], capture_output=True, text=True, timeout=float(
                self.config.get("inference_timeout_sec", 20.0)), check=False)
            if result.returncode:
                raise RuntimeError((result.stderr or "whisper.cpp failed").strip()[-300:])
            with open(output_json, "r", encoding="utf-8") as stream:
                return parse_whisper_result(json.load(stream))
        finally:
            for temporary in (path, output_json):
                try:
                    os.unlink(temporary)
                except FileNotFoundError:
                    pass

    def _run(self):
        reconnect = 2.0
        binary, model = self.config["whisper_binary"], self.config["model"]
        if not os.path.isfile(binary) or not os.access(binary, os.X_OK):
            try:
                device = discover_microphone(self.config.get("alsa_device", ""))
            except (OSError, RuntimeError, subprocess.SubprocessError):
                device = None
            self._status("ERROR", microphone_connected=device is not None, whisper_loaded=False,
                         device=device or "", error=f"Whisper binary not executable: {binary}")
            return
        if not os.path.isfile(model):
            try:
                device = discover_microphone(self.config.get("alsa_device", ""))
            except (OSError, RuntimeError, subprocess.SubprocessError):
                device = None
            self._status("ERROR", microphone_connected=device is not None, whisper_loaded=False,
                         device=device or "", error=f"Whisper model missing: {model}")
            return
        sample_rate = int(self.config.get("sample_rate", 16000))
        frame_ms, frame_bytes = 30, sample_rate * 2 * 30 // 1000
        silence_frames = max(1, int(float(self.config.get("silence_sec", 0.8)) * 1000 / frame_ms))
        max_frames = max(1, int(float(self.config.get("max_utterance_sec", 5.0)) * 1000 / frame_ms))
        min_bytes = int(float(self.config.get("min_utterance_sec", 0.25)) * sample_rate * 2)
        threshold = int(self.config.get("vad_rms", 450))
        while not self.stop_event.is_set():
            try:
                device = discover_microphone(self.config.get("alsa_device", ""))
                self.process = subprocess.Popen([
                    "arecord", "-q", "-D", device, "-t", "raw", "-f", "S16_LE",
                    "-r", str(sample_rate), "-c", "1",
                ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=-1)
                self._status("LISTENING", microphone_connected=True, whisper_loaded=True,
                             device=device)
                self._capture(frame_bytes, silence_frames, max_frames, min_bytes, threshold, sample_rate)
                if self.process.poll() not in (None, 0):
                    error = self.process.stderr.read().decode(errors="replace")[-300:]
                    raise RuntimeError(error or "arecord stopped")
            except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
                self._status("ERROR", microphone_connected=False, whisper_loaded=True, error=str(exc))
            finally:
                if self.process is not None and self.process.poll() is None:
                    self.process.terminate()
                    try:
                        self.process.wait(timeout=1)
                    except subprocess.TimeoutExpired:
                        self.process.kill()
                self.process = None
            self.stop_event.wait(reconnect)

    def _capture(self, frame_bytes, silence_frames, max_frames, min_bytes, threshold, sample_rate):
        pre_roll, utterance = deque(maxlen=6), []
        quiet = 0
        while not self.stop_event.is_set() and self.process.poll() is None:
            frame = read_exact(self.process.stdout, frame_bytes, self.process)
            active = audioop.rms(frame, 2) >= threshold
            if not utterance:
                pre_roll.append(frame)
                if active:
                    utterance = list(pre_roll)
                    quiet = 0
                continue
            utterance.append(frame)
            quiet = 0 if active else quiet + 1
            if len(utterance) < max_frames and quiet < silence_frames:
                continue
            pcm = b"".join(utterance[:-quiet] if quiet else utterance)
            utterance, quiet = [], 0
            if len(pcm) < min_bytes:
                continue
            self._status("PROCESSING", microphone_connected=True, whisper_loaded=True)
            started = time.monotonic()
            succeeded = False
            try:
                phrase, confidence = self._transcribe(pcm, sample_rate)
                threshold = float(self.config.get("min_mean_token_confidence", 0.55))
                command, reason = classify_transcript(
                    phrase, confidence, threshold, self.config.get("wake_phrase", "ROVER"))
                self.on_event({"phrase": phrase, "command": command, "recognized": bool(phrase),
                               "accepted": command is not None, "reason": reason,
                               "confidence": round(confidence, 3) if confidence is not None else None,
                               "inference_sec": round(time.monotonic() - started, 3)})
                succeeded = True
            except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as exc:
                self.on_event({"phrase": "", "command": None, "recognized": False,
                               "accepted": False, "reason": f"INFERENCE_ERROR: {exc}"})
                self._status("ERROR", microphone_connected=True, whisper_loaded=False,
                             error=str(exc))
            if succeeded:
                self._status("LISTENING", microphone_connected=True, whisper_loaded=True)
