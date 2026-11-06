# Modules (one Python file each)

Every module's opening docstring is its tiny in-file README. Files live together under `rover/`, with **no nested Python module folders**.

| File | Job |
|---|---|
| `vision.py` | Future consumer and validator of body-tracking observations |
| `zed_body_bridge.py` | ZED 2i native SDK body tracking publisher (perception only) |
| `gesture.py` | Mission-gated FOLLOW arm gesture, target selection and re-arming after tracking loss |
| `hammer.py` | Hammer detection / 3D target data (no grasp yet) |
| `voice.py` | Offline Whisper command parsing / future ASR hook |
| `nav.py` | Validated GNSS destinations and pure follower guidance; GNSS navigation adapter still pending |
| `mission.py` | Mock approach → FOLLOW → STAY → FETCH → COME → GIVE sequence; explicit pickup/release feedback and development approval |
| `safety.py` | Conservative movement-command gate |
| `hw.py` | Disabled placeholders for old motors, GNSS, Pi/EKF |
| `eva.py` | Future astronaut GNSS + EVA audio/video bridge |
| `gnss_imu_node.py` | N300 WP Pro IMU (supplied HiPNUC protocol) and separate NMEA GNSS ROS publishers; hardware validation pending |
| `hipnuc.py` | Pure CRC-checked HiPNUC decoder and sensor-to-ROS axes/unit conversion |
| `arm.py` | Future autonomous hammer pick / give |
| `mission_control_node.py` | Interactive destination intake with acknowledgement; navigation blocked pending drive interface; gesture-authorized FOLLOW diagnostics |
| `web.py` | Offline FastAPI dashboard, safe DEV simulator, and atomic `input.json` API |
| `demo.py` | Non-actuating offline/VM demonstration |
