# ROADMAP — Astronaut Assistance (0–50 points)

**Now:** phases 0–8; **later:** EVA monitoring and arm/gripper. Entries are **capability targets**, not actual scores or implementation claims.

| Phase | Build | File(s) | Test |
|---|---|---|---|
| 0 | Ubuntu 22.04 VM Humble package + mocked old hardware | `mission_control_node.py`, `hw.py`, `config.yaml` | colcon, mock events, no motor motion |
| 1 | ZED 2i native SDK bridge verified on Jetson | `zed_body_bridge.py` | CAMERA/IMAGE meters and capture timestamps; rover TF pending |
| 2 | Existing GNSS waypoint adapter (5 pts) | `nav.py`, `hw.py` | within 3 m in safe trials |
| 3 | Stage 2 perception and identity locking implemented | `vision.py` | 47 tests; live tracking/loss/disconnect verified; no actuation |
| 4 | FOLLOW motion, turns, stops | `nav.py`, `safety.py` | safe tracking left/right and stopping |
| 5 | STAY and local voice commands | `mission.py`, `voice.py` | latch remains stopped over >20 m separation |
| 6 | COME using fresh GNSS + visual reacquisition | `eva.py`, `nav.py` | within 3 m of astronaut |
| 7 | COME selection baseline implemented; full quiet beckoning/FOLLOW pending | `gesture.py`, `mission.py` | synthetic ROS confirmation passes; live recognition and false positives pending |
| 8 | Hammer detection + 3D staging + development YES | `hammer.py`, `mission.py` | stop before pickup, no arm command |
| 9 (later) | EVA video/mic monitoring (5 pts) | `eva.py` | C2 receives suit audio and video |
| 10 (later) | Autonomously pick hammer (10 pts) | `arm.py` | grasp and lift in trials |
| 11 (later) | Autonomously GIVE hammer (5 pts) | `arm.py` | safe autonomous deposit |

Other attainable points: 5 for GNSS approach, 15 for full FOLLOW, 5 for STAY, 5 for COME = **30** without EVA monitoring, FETCH and GIVE. EVA monitoring raises to 35. Full hardware/software behaviors raise the theoretical ceiling to 50. ALL scores require actual successful judged execution under the competition rules.

Do not call the phase complete before verifying tests on the appropriate VM/Jetson hardware.
