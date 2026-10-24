# Modules (one Python file each)

Every module's opening docstring is its tiny in-file README. Files live together under `rover/`, with **no nested Python module folders**.

| File | Job |
|---|---|
| `vision.py` | Future consumer and validator of body-tracking observations |
| `zed_body_bridge.py` | ZED 2i native SDK body tracking publisher (perception only) |
| `gesture.py` | Temporal astronaut beckoning recognition |
| `hammer.py` | Hammer detection / 3D target data (no grasp yet) |
| `voice.py` | Offline Whisper command parsing / future ASR hook |
| `nav.py` | GNSS and person-follow guidance adapters |
| `mission.py` | Safe mission state transitions / approval state |
| `safety.py` | Conservative movement-command gate |
| `hw.py` | Disabled placeholders for old motors, GNSS, Pi/EKF |
| `eva.py` | Future astronaut GNSS + EVA audio/video bridge |
| `arm.py` | Future autonomous hammer pick / give |
| `mission_control_node.py` | Development-only ROS 2 mission state node |
| `web.py` | Offline FastAPI dashboard, safe DEV simulator, and atomic `input.json` API |
| `demo.py` | Non-actuating offline/VM demonstration |
