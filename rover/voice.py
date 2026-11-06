"""Voice: parse offline laptop ASR into allowed commands. Parser is IMPLEMENTED.

Whisper.cpp adapter is FUTURE; never treat text as an unvalidated motor command.
"""
from typing import Optional

_COMMANDS = {'start','follow','stay','come','fetch','give','yes','no','stop'}
_ALIASES = {'go':'start', 'emergency stop':'stop'}


def parse(text: str) -> Optional[str]:
    normalized = ' '.join(text.lower().strip(' \t\n.!?').split())
    normalized = _ALIASES.get(normalized, normalized)
    return normalized if normalized in _COMMANDS else None


def transcribe_local(audio_path: str) -> str:
    raise NotImplementedError('Install/test Whisper.cpp locally on C2/VM first')
