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
to test again. An event does not activate FOLLOW or command motion. Confirm the
same ID remains valid using `ros2 topic echo /rover/vision/target --once`.
This wrist-motion rule is a development baseline; live recognition and false
positive rates remain unverified. Automated coverage: 64 tests plus a synthetic
ROS pipeline that overrides an initial lock and confirms exactly one COME event.
