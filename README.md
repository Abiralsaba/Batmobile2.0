# Mars Rover — Ubuntu VM / Jetson

**Develop on your Mac's Ubuntu 22.04 VM with ROS 2 Humble.** Later copy this same ROS 2 package to Jetson Orin with its compatible Ubuntu 22.04 / JetPack / ZED SDK. Do **not** develop natively in macOS for this project.

One Python file per feature under `rover/`; no nested Python subpackages. Module descriptions are in each file's opening docstring and in [MODULES.md](MODULES.md).

## Scope

- **Now:** existing GNSS + IMU + EKF + motors (integration placeholders); ZED X astronaut detection and stereo position (adapter placeholder); quiet beckoning; offline voice from C2; FOLLOW, STAY, COME; hammer detection and **YES/NO-authorized approach only**.
- **Later:** attach real prior rover drivers; EVA-suit monitoring; arm pickup and GIVE.
- **No LiDAR**, no cloud dependency at competition time. The current code is a **non-actuating scaffold**. No real ZED / Whisper / motors are connected.

## Quick start — Ubuntu 22.04 VM (ROS 2 Humble)

```bash
# Install ROS 2 Humble separately first, then in an Ubuntu terminal:
source /opt/ros/humble/setup.bash
mkdir -p ~/rover_ws/src
# Extract ZIP so this project is at ~/rover_ws/src/mars_rover
# (or rename that folder to mars_rover; package name stays mars_rover).
cd ~/rover_ws
python3 -m pip install --user -r src/mars_rover/requirements.txt
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch mars_rover mars_rover.launch.py
```

> Install `python3-colcon-common-extensions`, `python3-rosdep`, and `ros-humble-ros-base` / `ros-humble-desktop` as needed. Use system Python for ROS 2 to avoid mixing mismatched rclpy installations. Ubuntu ROS packages come from apt, not pip. See [DEPENDENCIES.md](DEPENDENCIES.md).

In a **second sourced terminal**:

```bash
ros2 topic echo /rover/state
ros2 topic pub --once /dev/cmd std_msgs/msg/String '{data: "start"}'
```

The launch file currently starts one executable, `mission_control_node`. Future
perception or hardware files will gain `_node.py` names only when they become
real ROS nodes; the existing feature modules remain pure, testable logic.

For a pure Python simulation (works in the VM with Python even without ROS):

```bash
cd ~/rover_ws/src/mars_rover
PYTHONPATH=. python3 -m rover.demo
PYTHONPATH=. python3 -m unittest discover -s tests -v
```

## Essential files

- [AGENT_PROMPT.md](AGENT_PROMPT.md): copy to your AI coding agent.
- [BRAIN.md](BRAIN.md): fixed project context / decisions.
- [ROADMAP.md](ROADMAP.md): ordered development stages.
- [MODULES.md](MODULES.md): one-line descriptions of each `.py` file.
- [requirements.txt](requirements.txt), [requirements-jetson.txt](requirements-jetson.txt), [DEPENDENCIES.md](DEPENDENCIES.md): Python vs ROS / CUDA / ZED SDK dependencies.
- [config.yaml](config.yaml): centralized, provisional topic names and conservative defaults.

## Safety and competition caveats

Never connect demo commands directly to motors. A physical all-power E-stop, low-level controller watchdog, calibrated `base_link` pose, local stereo-obstacle safety checks and reviewed motor adapter are required before physical driving. Laptop voice is a **development operator interface**, not a substitute for an astronaut's quiet beckoning gesture (full 15-point FOLLOW option) or other judged astronaut-issued commands. 2027 FETCH and GIVE still require real arm manipulation to earn their points.

## Stage 2: perception-only target locking

From `/home/arm/Documents/Batmobile`, source `/opt/ros/humble/setup.bash` and
`install/setup.bash` in each terminal, then run:

```bash
ros2 run mars_rover zed_body_bridge
ros2 run mars_rover vision
ros2 topic echo /rover/vision/target
```

Alternatively, `ros2 launch mars_rover vision.launch.py start_bridge:=true` starts
both. The default `start_bridge:=false` starts vision alone. Run only one bridge;
do not run the ZED ROS wrapper alongside it. The mission launch is unchanged.

Input is `/rover/vision/bodies`; output is strict JSON on `/rover/vision/target`
(`std_msgs/msg/String`). `valid` means an eligible perception observation, not
permission to move. No actuator publisher is present. Startup selects only a sole
person after 3 consecutive OK frames; multiple people require explicit selection.
Any bad/lost frame invalidates immediately. Recovery of the same ID requires 3 OK
frames. A camera-message gap over 0.5 s retains the ID but latches reselection:
IDs may be reused across bridge restarts. Reset or select after the stream resumes:

```bash
ros2 topic pub --once /rover/vision/select std_msgs/msg/String '{data: "{\"action\":\"reset\"}"}'
# Replace 49 with an ID actually present in /rover/vision/bodies:
ros2 topic pub --once /rover/vision/select std_msgs/msg/String '{data: "{\"action\":\"select\",\"person_id\":49}"}'
```

Startup ROS parameters are declared in `VisionConfig` in `rover/vision.py`:
`acquisition_frames`, `reacquisition_frames`, `tracking_timeout_sec`,
`max_message_age_sec`, `future_tolerance_sec`, `min_forward_m`, `max_range_m`,
`median_window`, `ema_alpha`, `jump_margin_m`, `max_speed_mps`, `publish_hz`.
For example, append `--ros-args -p acquisition_frames:=4 -p ema_alpha:=0.5`.
The watchdog uses steady time even if ROS simulated time is paused.

The frame name `zed_left_camera_frame_optical` explicitly denotes SDK CAMERA /
IMAGE coordinates: X right, Y down, Z forward, meters. It does **not** advertise a
TF to `base_link`. Distance is horizontal X-Z range; bearing is positive right.
Distance and lateral position use a rolling median, with optional EMA; filtered Z
and bearing are derived consistently. Keypoints are unsmoothed camera coordinates.

Future `gesture.py` should consume valid BODY_18 `keypoints_m`, per-side arm joint
quality, and distinct `observation_seq` / `timestamp` values. Null coordinates
remain null. Future `nav.py` must consume only this validated target, check age,
ID and `valid`, and use a calibrated camera-to-rover transform plus independent
safety arbitration. Repeated 10 Hz publications retain the observation timestamp
and sequence; `published_timestamp` is not evidence of a new detection. Invalid
outputs clear geometry and joint quality. No gesture or navigation integration is
included in this stage.

## COME gesture selection (development baseline)

After sourcing ROS and this workspace in each terminal, run one process per line:

```bash
ros2 run mars_rover zed_body_bridge
ros2 run mars_rover vision
ros2 run mars_rover gesture   # python3 -m rover.gesture also works
ros2 topic echo /rover/gesture/event
```

Start the event subscriber before gesturing. Stand 2–3 meters away with the entire
arm visible. Move the wrist below the elbow → above it → below it. Either arm is
supported using BODY_18 joint ordering. The node monitors every tracked person,
requests that person's ID on `/rover/vision/select`, then waits up to 3 seconds
for a fresh, valid vision lock before emitting one `COME` event. Simultaneous
completions by different people are rejected. Missing joints, invalid/stale raw
frames, tracking loss and camera gaps clear the current gesture attempt.

There is only one confirmed event per gesture-node run; restart **gesture only**
to test again. With Stage 4 mission control staged in WAIT_COMMAND, a confirmed
event activates diagnostic FOLLOW guidance. Confirm the
same ID remains valid using `ros2 topic echo /rover/vision/target --once`.
This wrist-motion rule is a development baseline; live recognition and false
positive rates remain unverified. Automated coverage: 64 tests plus a synthetic
ROS pipeline that overrides an initial lock and confirms exactly one COME event.

## Stage 4: motor-disabled 2-meter FOLLOW

Build the single package in its checkout:

```bash
cd /home/arm/Documents/Batmobile
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mars_rover --executor sequential
source install/setup.bash
ros2 launch mars_rover mars_rover.launch.py
```

That launch starts only the existing mission controller. For **operator-run manual
camera testing**, source ROS and this installation in each terminal, then run
one process per terminal (skip any already running):

```bash
ros2 run mars_rover zed_body_bridge
ros2 run mars_rover vision
ros2 run mars_rover gesture
ros2 topic echo /rover/follow/guidance
```

Use exactly one camera owner; never start the ZED wrapper alongside the bridge.
Before performing COME, stage the mission in a separate sourced terminal:

```bash
ros2 topic pub --once /dev/cmd std_msgs/msg/String '{data: "start"}'
ros2 topic pub --once /dev/cmd std_msgs/msg/String '{data: "arrived"}'
ros2 topic echo /rover/state --once   # Must report WAIT_COMMAND.
```

One fresh confirmed COME for vision's valid locked ID calls the existing `follow`
transition. Guidance continues without another gesture. The `come` mission event
retains its previous meaning. Mock `/dev/cmd follow` still changes the mock FSM,
but cannot authorize nonzero diagnostic guidance without a COME activation.

`/rover/follow/guidance` uses `std_msgs/msg/String` with strict JSON: `state`,
`person_id`, `distance_m`, `desired_m`, `distance_error_m`, `tracking_valid`,
`linear_mps`, `angular_rps`, `reason`, `source_frame`, `calibration_status`,
`motors_enabled`, and `reset_required`. Unavailable distances/errors are null.
Guidance is published on input changes and a 20 Hz steady-clock watchdog.

Distance is the filtered camera X-Z range. Desired range is 2.0 m; the inclusive
1.85–2.15 m band suppresses forward guidance. Closer targets never cause reverse.
Suggested speeds are capped at 0.20 m/s and 0.30 rad/s; heading errors above
0.5 rad suppress forward guidance while turning. The approximate mapping is
forward = camera Z, left = -camera X. Every result is marked
`CALIBRATION_REQUIRED` and `motors_enabled: false`; no actuator topic is published.
No calibrated camera-to-base_link transform, obstacle avoidance or driving
authorization is supplied by this stage. Physically measuring distance, bearing,
recovery, and gesture false positives is reserved for the operators.

Invalid/lost/SEARCHING/OFF targets or another ID immediately zero guidance.
Recovery of the authorized ID uses vision's existing consecutive-frame check.
Repeated timestamps/sequences cannot renew freshness. Observation expiry is
0.5 s, checked every 0.05 s and on callbacks; it also works when ROS time pauses.
Vision disconnect/reselection, sequence rollback, or target-topic silence beyond
0.5 s revoke authorization and enter SAFE_STOP. Then send `/dev/cmd` events
`reset`, `start`, `arrived`, restart **gesture only**, and perform a new COME.
Gesture requests a new vision selection; a camera restart alone cannot authorize
FOLLOW. STOP/STAY commands immediately clear authorization. Because gesture is
one-shot, restart it whenever another COME activation is needed, including after
an event was ignored outside WAIT_COMMAND.

Software-only verification (no camera opened; dedicated ROS domain):

```bash
source /opt/ros/humble/setup.bash
source /home/arm/Documents/Batmobile/install/setup.bash
cd /home/arm/Documents/Batmobile
ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1 BATMOBILE_FOLLOW_ROS_TEST=1 \
  python3 -m unittest discover -s tests -v
```

The opt-in tests start only mission control, vision and gesture, with synthetic
observations. Do not run other nodes on the test domain. Ordinary test discovery
skips these two transport tests; the pure policy tests need no camera or ROS.

## Stage 5: original GNSS/N100 driver and Pi diagnostics

`gnss_imu_node.py` integrates the supplied real `N100HeadingDriver` and
`GNSSIMUNode`; `pi_bridge_demo.py` only inspects FOLLOW guidance. Neither starts
automatically. Build/source using the Stage 4 commands above. Added dependencies:
ROS `sensor_msgs`/`geometry_msgs`, Python `pyserial>=3.5,<4` and `pynmea2>=1.15,<2`
(installed as 3.5 and 1.15.0). No unrelated libraries were changed.

For **manual sensor tests**, first identify the actual devices without opening them:

```bash
python3 -m serial.tools.list_ports -v
# Replace these placeholders with the verified persistent serial paths:
ros2 run mars_rover gnss_imu_node --ros-args \
  -p imu_port:=/dev/serial/by-id/REPLACE_WITH_IMU_ID \
  -p gnss_port:=/dev/serial/by-id/REPLACE_WITH_GNSS_ID
```

Defaults: `imu_baud=921600`, `gnss_baud=115200`, `imu_frame_id=imu_link`,
`gnss_frame_id=gps_link`, `sensor_timeout_sec=2.0`. Empty ports use unique VID/PID
discovery: `imu_vid=6790`, `gnss_vid=5446`, `imu_pid/gnss_pid=-1` (unspecified).
Multiple matches are refused. Actual IDs remain unverified: no devices were
enumerated, and the older `~/gps_node.py` uses IMU VID 4292. Prefer explicit paths.
Keep host UTC synchronized for NMEA freshness checks. Parameters are read-only;
restart the node after disconnection or a device timestamp-counter restart.

In separate sourced terminals:

```bash
ros2 topic echo /imu/data --once    # sensor_msgs/msg/Imu, imu_link
ros2 topic echo /nav_sat --once     # sensor_msgs/msg/NavSatFix, gps_link
ros2 run mars_rover pi_bridge_demo
ros2 topic echo /rover/demo/pi_bridge/status   # std_msgs/msg/String
```

IMU orientation is normalized and published once per new AHRS sample, preserving
`q_z * (q_rr * raw_q)` and magnetometer scale `1e-7`. Stamps are receipt times;
covariance is unknown (zeros); absent angular velocity/acceleration have
`covariance[0]=-1`. Heading stays internal/debug: at zero tilt +mag X = 0°,
+mag Y = 90°. North/clockwise mounting alignment and declination remain unverified.

Protocol limitation: original `FC + 6 header bytes + payload + FD`, type `0x40`
with `<12fq` (56 bytes), and type `0x41` with `<10fq` (48 bytes) are retained.
Length/type/end-marker checks and resynchronization are tested, **CRC is not**.
The [vendor AHRS table](https://doc.fdisystems.cn/FQA/02-fdilink/03-data-packets/MSG_AHRS.html)
agrees on quaternion offsets but lists a different packet ID; the exact N100
manual/captured frames are needed to verify firmware and CRC. Trailing device
timestamps are treated as monotonic counters for replay rejection.

NMEA checksums are required. Invalid fixes publish `STATUS_NO_FIX`/NaN coordinates;
missing/stale/malformed input produces no sample and reports status in logs.
Equatorial fixes are accepted. GGA altitude is MSL + geoid separation; missing
height/geoid or RMC yields NaN altitude. No cached altitude or invented covariance.

The demo consumes `/rover/follow/guidance` and publishes only the String diagnostic
status above. It validates structure/ranges and expires receipt freshness at 0.5 s.
Current guidance lacks source timestamps: `source_freshness_verified=false`;
pre-arrival delay/replays cannot be established. Optional timestamps are checked.
`action_allowed`, `pi_connected`, `motors_enabled`, `command_executed` stay false.
There are no Pi sockets, CAN or actuator publishers.

All 133 software tests pass, using mocked serial/synthetic data; physical tests
remain manual. Verify identity, packet integrity, mounting, fix/altitude and
unplug/freshness behavior. Future Jetson → Pi → controller → motors integration
needs the real Pi/MIT/CAN/transport code, IDs, kinematics, odometry and E-stop/watchdog.
