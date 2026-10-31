# Batmobile EVA Suit

Offline Raspberry Pi EVA application for Ubuntu 24.04 and ROS 2 Jazzy. It reads
the Beitian B9 GNSS, the ESP32-S3 three-finger glove and a USB microphone. It
publishes validated command requests but does not control rover motors.

The rover Jetson remains on ROS 2 Humble. A visible rover topic is required
before the EVA backend accepts FOLLOW or STOP, and command success requires a
real `/eva/command/ack`. Do not bypass these checks.

## Raspberry Pi installation

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep python3-venv \
  python3-pyqt5 python3-serial python3-yaml alsa-utils

cd ~/Documents/Batmobile
python3 -m venv --system-site-packages .venv
.venv/bin/pip install 'pynmea2>=1.15,<2' 'pyserial>=3.5,<4' 'PyYAML>=5.4'

source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths eva_suit_pkg --ignore-src -r -y
colcon build --base-paths eva_suit_pkg --packages-select eva_suit --symlink-install
source install/setup.bash
```

Keep deployed hardware configuration outside the Git checkout so a pull cannot
overwrite working serial paths:

```bash
sudo install -d -m 0755 /etc/eva-suit
if [[ ! -e /etc/eva-suit/eva.yaml ]]; then
  sudo install -m 0644 eva_suit_pkg/config/eva-pi.example.yaml /etc/eva-suit/eva.yaml
fi
sudoedit /etc/eva-suit/eva.yaml
```

The example contains the physically identified GNSS and ESP32 `/dev/serial/by-id`
paths, 115200 baud, `plughw:2,0`, and the `small.en` model path. Device indexes can
change; recheck them with:

```bash
ls -l /dev/serial/by-id/
python3 -m serial.tools.list_ports -v
arecord -l
test -s /opt/eva/models/ggml-small.en.bin
```

Never claim the model is installed solely because its path appears in YAML.

## Backend, GUI and `eva` shortcut

```bash
sudo install -m 0644 eva_suit_pkg/deploy/eva-suit-backend@.service \
  /etc/systemd/system/eva-suit-backend@.service
sudo install -m 0755 eva_suit_pkg/deploy/eva /usr/local/bin/eva
sudo install -m 0755 eva_suit_pkg/deploy/eva-gui-launcher \
  /usr/local/bin/eva-gui-launcher
sudo systemctl daemon-reload
sudo systemctl enable --now "eva-suit-backend@${USER}.service"
```

The shortcut intentionally uses `set -eo pipefail`, not `set -u`, because ROS 2
Jazzy setup scripts reference optional unset variables.

```text
eva          start the backend if needed and show compact live telemetry
eva stop     stop the systemd service or EVA-managed fallback process
eva restart  restart it and show status
eva status   show service, ROS node and configuration state
eva topics   list /eva/* topics
eva logs     follow service or fallback logs
```

Closing the `eva` monitor does not stop the backend. Set `EVA_WORKSPACE`,
`EVA_VENV`, or `EVA_CONFIG` only when using nonstandard locations.

## ESP32-S3 glove

Firmware: `firmware/flex/flex.ino`. It uses thumb/index/middle on GPIO 1/2/3.
The glove must be straight after boot before it arms. Three stable samples above
60% emit one `FOLLOW_START`; unbending any finger below 35% for three samples
emits one `FOLLOW_STOP`. Holding a gesture never repeats a command.

```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 \
  --board-options USBMode=hwcdc,CDCOnBoot=cdc eva_suit_pkg/firmware/flex
arduino-cli upload --port /dev/ttyACM1 --fqbn esp32:esp32:esp32s3 \
  --board-options USBMode=hwcdc,CDCOnBoot=cdc eva_suit_pkg/firmware/flex
```

The backend validates `FLEX_JSON_V2`, suppresses duplicate edges and requests a
one-time STOP if glove input becomes stale or disconnects.

## Voice recognition

Only exact `ROVER COME`, `ROVER FOLLOW` and `ROVER STOP` commands are eligible.
The wake phrase, lexical-token confidence and backend allowlist are all required.
Blank audio, missing token confidence, noise descriptions and approximate phrases
are rejected. Recordings are temporary and deleted after inference.

Benchmark `small.en` on the Pi before keeping it as the field model:

```bash
arecord -D plughw:2,0 -f S16_LE -r 16000 -c 1 -d 5 /tmp/eva-voice.wav
/usr/bin/time -v /opt/whisper.cpp/build/bin/whisper-cli \
  -m /opt/eva/models/ggml-small.en.bin -f /tmp/eva-voice.wav -l en
rm -f /tmp/eva-voice.wav
```

Record multiple samples of each supported phrase plus silence and background
noise. Measure acceptance, false acceptance, inference time and peak memory.
Do not move to a larger model without those measurements.

## ROS topics and diagnostics

Primary outputs are `/eva/gnss/fix`, `/eva/gnss/status`, `/eva/flex/raw`,
`/eva/flex/status`, `/eva/voice/event`, `/eva/voice/status`,
`/eva/command/request`, `/eva/command/status` and `/eva/system/status`.
The backend consumes `/eva/command/ack` and existing rover status topics.

```bash
ros2 topic echo /eva/system/status
ros2 topic echo /eva/command/status
ros2 topic info -v /rover/state
```

For Jazzy-to-Humble testing, use the intended identical `ROS_DOMAIN_ID`, keep
`ROS_LOCALHOST_ONLY=0`, verify DDS discovery in both directions and test QoS.
Ping alone does not prove ROS communication. Rover command reception,
acknowledgements, heartbeat and watchdog integration remain separate work.

## Tests

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m unittest discover -s eva_suit_pkg/test -v
python3 -m compileall -q eva_suit_pkg/eva_suit eva_suit_pkg/test
bash -n eva_suit_pkg/deploy/eva eva_suit_pkg/deploy/eva-gui-launcher
```

Hardware status must always be reported from observed data. Tests and mock data
must not be presented as physical validation.
