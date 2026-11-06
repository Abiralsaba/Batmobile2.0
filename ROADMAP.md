# ROADMAP — Astronaut Assistance (0–50 points)

**Now:** phases 0–8; **later:** EVA monitoring and arm/gripper. Entries are **capability targets**, not actual scores or implementation claims.

| Phase | Build | File(s) | Test |
|---|---|---|---|
| 0 | Ubuntu 22.04 VM Humble package + mocked old hardware | `mission_control_node.py`, `hw.py`, `config.yaml` | colcon, mock events, no motor motion |
| 1 | ZED 2i native SDK bridge verified on Jetson | `zed_body_bridge.py` | CAMERA/IMAGE meters and capture timestamps; rover TF pending |
| 2 | Existing GNSS waypoint adapter (5 pts) | `nav.py`, `hw.py` | within 3 m in safe trials |
| 3 | Stage 2 perception and identity locking implemented | `vision.py` | 47 tests; live tracking/loss/disconnect verified; no actuation |
| 4 | Stage 4: 2 m FOLLOW diagnostics implemented, motors disabled | `mission_control_node.py`, `nav.py` | software/synthetic ROS tests; manual camera calibration and physical evaluation pending |
| 5 | STAY and local voice commands | `mission.py`, `voice.py` | latch remains stopped over >20 m separation |
| 6 | COME using fresh GNSS + visual reacquisition | `eva.py`, `nav.py` | within 3 m of astronaut |
| 7 | FOLLOW arm-gesture selection and re-arming after tracking loss implemented | `gesture.py`, `mission.py`, `mission_control_node.py` | policy/callback recovery tests pass; current live ROS and suited-astronaut evaluation pending |
| 8 | Hammer detection + 3D staging + development YES | `hammer.py`, `mission.py` | stop before pickup, no arm command |
| 9 (later) | EVA video/mic monitoring (5 pts) | `eva.py` | C2 receives suit audio and video |
| 10 (later) | Autonomously pick hammer (10 pts) | `arm.py` | grasp and lift in trials |
| 11 (later) | Autonomously GIVE hammer (5 pts) | `arm.py` | safe autonomous deposit |

Other attainable points: 5 for GNSS approach, 15 for full FOLLOW, 5 for STAY, 5 for COME = **30** without EVA monitoring, FETCH and GIVE. EVA monitoring raises to 35. Full hardware/software behaviors raise the theoretical ceiling to 50. ALL scores require actual successful judged execution under the competition rules.

Do not call the phase complete before verifying tests on the appropriate VM/Jetson hardware.

## Mission sequence update — 2026-11-06

The phase table is a development plan; execution order is initial approach →
FOLLOW → STAY → FETCH → COME → GIVE. This order is now enforced by the mock
state machine, offline demo and DEV dashboard. COME waits for pickup confirmation;
GIVE waits for COME arrival; completion waits for ground-release confirmation.
The existing YES/NO development approach gate remains. STOP, stage-skipping
rejection and FOLLOW recovery have software tests. Navigation, arm actions and
authenticated astronaut command delivery are still pending hardware integration.

Coordinate startup now prompts for latitude/longitude and acknowledges receipt in
the ROS mission controller. It remains blocked in READY until the user supplies
the drive/navigation interface. The destination request path does not simulate
arrival or send motor commands. Policy tests cover validation, replay, busy-state
rejection and STOP/reset; the new native ROS transport test needs the Jetson/VM.
