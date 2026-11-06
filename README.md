# Mars Rover — Ubuntu VM / Jetson

**Develop on your Mac's Ubuntu 22.04 VM with ROS 2 Humble.** Later copy this same ROS 2 package to Jetson Orin with its compatible Ubuntu 22.04 / JetPack / ZED SDK. Do **not** develop natively in macOS for this project.

One Python file per feature under `rover/`; no nested Python subpackages. Module descriptions are in each file's opening docstring and in [MODULES.md](MODULES.md).

## Scope

- **Now:** existing GNSS + IMU + EKF + motors (integration placeholders); ZED X astronaut detection and stereo position (adapter placeholder); quiet beckoning; offline voice from C2; FOLLOW, STAY, COME; hammer detection and **YES/NO-authorized approach only**.
- **Later:** attach real prior rover drivers; EVA-suit monitoring; arm pickup and GIVE.
- **No LiDAR**, no cloud dependency at competition time. The current code is a **non-actuating scaffold**. No real ZED / Whisper / motors are connected.

## Enter the initial destination

After copying these changes to the Jetson, rebuild `mars_rover` and source its
installation. Start the controller in one terminal:

```bash
ros2 launch mars_rover mars_rover.launch.py
```

In another terminal with ROS and the same workspace sourced, run:

```bash
ros2 run mars_rover start_mission
```

It asks for **latitude**, then **longitude**, in WGS84 decimal degrees. Invalid
values are rejected and prompted again. Ctrl+C cancels input. The client sends
the destination to the controller and waits for an acknowledgement. It requires
exactly one controller and reports an error if no acknowledgement arrives.

**Current limitation:** the drive/navigation interface has not been supplied.
The controller receives the coordinates but stays in READY, reports
`NAVIGATION_UNAVAILABLE`, and sends no movement command. The client exits with
code 2 for this blocked state. This is not completed coordinate navigation.
Coordinates are held only in controller memory; they are cleared by STOP/reset
or restart and do not update the dashboard's separate `input.json` simulation.
Requests during an active mission, stale/replayed requests and malformed inputs
are rejected. No destination starts automatically when an interface becomes available.

The new String topics are `/rover/mission/start_request` and
`/rover/mission/start_status`. The outstanding integration needs the actual
Jetson/Pi ROS topic, action and service lists, then verified localization,
navigation dispatch/cancellation, freshness and arrival checks. `/dev/cmd start`
and `arrived` below remain simulation commands, not the coordinate startup path.

## Mission sequence — development simulation

The mission order is **initial approach → FOLLOW → STAY → FETCH → COME → GIVE**.
Run `python3 -m rover.demo` for the complete simulated sequence:

```text
start → arrived → follow → stay → fetch → found → yes → near
      → pickup_confirmed → come → arrived → give → release_confirmed
```

`yes` is the existing development approval for hammer approach; `no` returns
to STAY so FETCH can be retried. Autonomous hammer search, approach, pickup and
ground release still need implementation. COME is accepted only after simulated
pickup confirmation. Its arrival enters WAIT_GIVE; only a separate release
confirmation after GIVE reaches MISSION_COMPLETE. These inputs simulate feedback,
not successful hardware actions or competition points.

Tracking loss during FOLLOW returns to WAIT_COMMAND for a fresh FOLLOW gesture
and matching vision lock. Reaching the following distance does not end FOLLOW;
STAY ends it. A gesture cannot restart following during later mission stages.
STOP overrides every stage and requires reset before a new mission.

The dashboard exposes these events only in DEV; LIVE controls remain disabled.
`/dev/cmd` is likewise a mock interface. Navigation arrival checks, authenticated
astronaut commands, verified arm feedback and the Jetson-to-drive-Pi interface
remain to be connected. No motor output has been added.

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

## FOLLOW gesture selection (development baseline)

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
for a fresh, valid vision lock before emitting a `FOLLOW` event. Simultaneous
completions by different people are rejected. Missing joints, invalid/stale raw
frames, tracking loss and camera gaps clear the current gesture attempt.

The detector arms only while fresh `/rover/follow/guidance` reports
`follow_gesture_allowed=true` in WAIT_COMMAND. Run the mission controller too,
and stage it with `start`, `arrived` as shown below. A confirmed event activates
diagnostic FOLLOW guidance. Confirm the
same ID remains valid using `ros2 topic echo /rover/vision/target --once`.
This wrist-motion rule is a development baseline; live recognition and false
positive rates remain unverified. Automated coverage includes policy/callback
tests and an opt-in synthetic ROS pipeline for selection and recovery. After confirmation the
detector clears both arms' partial motions and enforces a one-second cooldown.
During active FOLLOW, STOP or STAY it cannot select a different person. After
tracking loss, a new complete gesture can reselect the astronaut (including a
new SDK tracking ID) and activate FOLLOW again without restarting any node.

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
Before the first FOLLOW gesture, stage the mission in a separate sourced terminal:

```bash
ros2 topic pub --once /dev/cmd std_msgs/msg/String '{data: "start"}'
ros2 topic pub --once /dev/cmd std_msgs/msg/String '{data: "arrived"}'
ros2 topic echo /rover/state --once   # Must report WAIT_COMMAND.
```

One fresh confirmed FOLLOW for vision's valid locked ID calls the existing `follow`
transition. Guidance continues without another gesture. The `come` mission event
retains its previous meaning. Mock `/dev/cmd follow` still changes the mock FSM,
but cannot authorize nonzero diagnostic guidance without a FOLLOW activation.

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

Invalid/lost/SEARCHING/OFF targets, another ID, sequence rollback or a camera
disconnect immediately zero guidance and revoke the old FOLLOW authorization.
The controller returns to WAIT_COMMAND with reason FOLLOW_GESTURE_REQUIRED.
Seeing a person again does not resume following: perform a fresh arm gesture,
then vision must confirm a fresh stable lock. No node restart or reset/start/arrived
sequence is needed for tracking-loss recovery. A new SDK ID is accepted only
through that new gesture and lock, never by automatic person switching.
Repeated timestamps/sequences cannot renew freshness. Observation expiry is
0.5 s, checked every 0.05 s and on callbacks; it also works when ROS time pauses.
STOP and STAY still block FOLLOW gestures; they are not tracking-loss recovery.
Explicit STOP still requires `reset`, `start`, `arrived` before a new gesture.
The real mission `come` command remains separate from this renamed visual event.

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

## Stage 5: GNSS / N300 WP Pro driver and Pi diagnostics

`gnss_imu_node.py` targets the WHEELTEC N300 WP Pro using the HiPNUC protocol
in the operator-supplied reader/parser files, through the existing
`N100HeadingDriver` and `GNSSIMUNode` classes; `hipnuc.py` contains the tested
decoder. `pi_bridge_demo.py` only inspects FOLLOW guidance. Neither node starts
automatically. Build/source using the Stage 4 commands above. Added dependencies:
ROS `sensor_msgs`/`geometry_msgs`, Python `pyserial>=3.5,<4` and `pynmea2>=1.15,<2`
(installed as 3.5 and 1.15.0). No unrelated libraries were changed.

`N100HeadingDriver` is retained only as a legacy Python class name. The model
name alone does not establish firmware protocol, baud rate or output axes;
confirm these against the supplied reader's live output on this device.

For **manual sensor tests**, first identify the actual devices without opening them:

```bash
python3 -m serial.tools.list_ports -v
# Replace these placeholders with the verified persistent serial paths:
ros2 run mars_rover gnss_imu_node --ros-args \
  -p imu_port:=/dev/serial/by-id/REPLACE_WITH_IMU_ID \
  -p imu_baud:=115200 -p imu_axes:=rfu \
  -p gnss_port:=/dev/serial/by-id/REPLACE_WITH_GNSS_ID
```

Defaults: `imu_baud=115200` (matching the uploaded reader), `imu_axes=rfu`,
`gnss_baud=115200`, `imu_frame_id=imu_link`,
`gnss_frame_id=gps_link`, `sensor_timeout_sec=2.0`. Empty ports use unique VID/PID
discovery: `imu_vid=6790`, `gnss_vid=5446`, `imu_pid/gnss_pid=-1` (unspecified).
Multiple matches are refused. Actual IDs remain unverified: no devices were
enumerated, and the older `~/gps_node.py` uses IMU VID 4292. Prefer explicit paths.
Keep host UTC synchronized for NMEA freshness checks. Parameters are read-only;
restart the node after disconnection or an HI91 timestamp rollback (sensor
restart, counter rollover, or time synchronization). Identical/backward HI91
timestamps never refresh a sample. HI92 and HI81 have no reliable monotonic
counter in this integration; freshness is based on receipt of CRC-valid frames.

In separate sourced terminals:

```bash
ros2 topic echo /imu/data --once    # sensor_msgs/msg/Imu, imu_link
ros2 topic echo /nav_sat --once     # sensor_msgs/msg/NavSatFix, gps_link
ros2 run mars_rover pi_bridge_demo
ros2 topic echo /rover/demo/pi_bridge/status   # std_msgs/msg/String
```

IMU input is now `5A A5`, little-endian payload length, CRC-16/XMODEM, payload.
HI91 (76 bytes), HI92 (48 bytes), and HI81 (104 bytes) are decoded using the
uploaded parser's layouts. Length and CRC are checked before processing; malformed
or unknown items cannot produce partial samples. The uploaded reference files
remain unchanged; the installed node does not depend on their uninstalled
`parsers/` folder or its shared mutable frame object.

Orientation, angular velocity and acceleration come from one validated sample.
The existing 10 Hz timer publishes the newest sample at most once, so higher-rate
input is downsampled. Quaternions are normalized; all covariances remain unknown
(zeros). Angular rates are rad/s and acceleration is m/s², including gravity.
The old N100 rotations and raw-magnetometer heading calculation are removed.

`imu_axes=rfu` assumes the sensor output is X right, Y forward, Z up. It converts
vectors to ROS front-left-up as `(y, -x, z)` and applies the corresponding **body**
rotation to the WXYZ quaternion. `imu_axes=flu` leaves already-ROS output axes
unchanged. Verify firmware configuration and mounting before selecting this
parameter: it does not configure the hardware or establish a TF to `base_link`.
The [HiPNUC IMU manual](https://download.hipnuc.com/en/products/imu/cum.html)
documents the coordinate and unit conventions. The uploaded monitor's
`(-yaw) % 360` agrees with RFU HI91/HI92 heading; already-FLU yaw uses
`(90 - yaw) % 360`. Heading remains debug-only, and may be relative or magnetic
depending on the sensor mode; no true-north accuracy or declination is invented.

HI81 is an INS format: its yaw is already clockwise compass heading, and its
RFU attitude is accepted only in navigation/dead-reckoning states 3/6. State 6
does not imply an aided or accurate GNSS solution. Offset 90 is reserved in the
[current INS manual](https://download.hipnuc.com/en/products/ins/cum.html), so it
is not used as the timestamp despite the older uploaded parser's interpretation.
Binary INS positions do not replace the separate NMEA GNSS input.

Portable protocol tests: `python3 -m unittest discover -s tests -p 'test_hipnuc.py' -v`.
ROS wrapper tests (mocked serial): in a sourced ROS environment run
`python3 -m unittest discover -s tests -p 'test_gnss_imu.py' -v`.
N300 WP Pro hardware/ROS verification is pending: check frame type, baud, mounting,
stationary gravity, clockwise/counterclockwise turns, and unplug/freshness behavior.

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

The earlier N100 integration passed 133 software tests with mocked serial/synthetic
data; that historical count does not verify the replacement N300 WP Pro hardware.
Verify identity, packet integrity, mounting, fix/altitude and
unplug/freshness behavior. Future Jetson → Pi → controller → motors integration
needs the real Pi/MIT/CAN/transport code, IDs, kinematics, odometry and E-stop/watchdog.
