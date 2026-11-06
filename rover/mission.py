"""Mission: mock state-machine logic with strict event transitions. IMPLEMENTED (no actuation).

Inputs: trusted simulated events. Output: state changes; no motor publications.
"""
from enum import Enum


class State(str, Enum):
    READY = 'READY'
    GO = 'GO_TO_ASTRONAUT'
    WAIT = 'WAIT_COMMAND'
    FOLLOW = 'FOLLOW'
    STAY = 'STAY_LOCKED'
    COME = 'COME'
    FIND = 'FIND_HAMMER'
    ASK = 'WAIT_APPROVAL'
    NEAR = 'APPROACH_HAMMER'  # STATE ONLY; not an actual approach
    ARM = 'FUTURE_ARM_PICKUP'
    WAIT_COME = 'WAIT_COME'
    WAIT_GIVE = 'WAIT_GIVE'
    GIVE = 'FUTURE_ARM_RELEASE'
    DONE = 'MISSION_COMPLETE'
    STOP = 'SAFE_STOP'


_NEXT = {
    (State.READY, 'start'): State.GO,
    (State.GO, 'arrived'): State.WAIT,
    (State.WAIT, 'follow'): State.FOLLOW,
    (State.FOLLOW, 'tracking_lost'): State.WAIT,
    (State.FOLLOW, 'stay'): State.STAY,
    (State.STAY, 'fetch'): State.FIND,
    (State.FIND, 'found'): State.ASK,
    # Keep the existing development-only approval gate until safe autonomous
    # hammer approach is implemented. These transitions do not execute actions.
    (State.ASK, 'yes'): State.NEAR,
    (State.ASK, 'no'): State.STAY,
    (State.NEAR, 'near'): State.ARM,
    (State.ARM, 'pickup_confirmed'): State.WAIT_COME,
    (State.WAIT_COME, 'come'): State.COME,
    (State.COME, 'arrived'): State.WAIT_GIVE,
    (State.WAIT_GIVE, 'give'): State.GIVE,
    (State.GIVE, 'release_confirmed'): State.DONE,
    (State.STOP, 'reset'): State.READY,
}


class Mission:
    def __init__(self):
        self.state = State.READY
        self.history = []

    def event(self, name: str, *, source: str = 'dev') -> bool:
        """Simulate an event; a real robot needs authenticated, verified feedback.

        Arrival, pickup and release confirmations are mock inputs here, never
        inferred from issuing a command or from time passing.
        """
        name = name.strip().lower()
        previous = self.state
        if name == 'stop':
            next_state = State.STOP
        else:
            next_state = _NEXT.get((previous, name))
        if next_state is None:
            return False
        self.state = next_state
        self.history.append((previous.value, name, self.state.value, source))
        return True
