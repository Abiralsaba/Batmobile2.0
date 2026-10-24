# Dependency rules — Ubuntu VM development and Jetson runtime

## Ubuntu 22.04 VM on Mac
- Install Ubuntu **22.04**, ROS **2 Humble**, `python3-colcon-common-extensions`, `python3-rosdep`, `python3-pip` and tools via Ubuntu apt/ROS repos. Use `rosdep install --from-paths src --ignore-src -r -y` for package dependencies.
- `requirements.txt` holds only small *pip-installable Python* dependencies. The standard-library unit tests require no GPU.
- For native ROS 2 Humble Python nodes, use the Python installation compatible with the apt-installed ROS 2 environment; generally Ubuntu 22.04 Python 3.10. Do not `pip install rclpy` into an unrelated Mac Python runtime.
- VM can run mock/recorded data; ZED X GMSL2 camera and Jetson TensorRT inference will **not** be assumed accessible through UTM VM.

## Jetson Orin Nano Super (verified 2026-10-25)
- Runtime: Ubuntu 22.04.5, ARM64, JetPack 6.2.2 / L4T R36.5.0, Python 3.10 and ROS 2 Humble.
- Camera: ZED 2i USB, ZED SDK 5.2.3 and its installed `pyzed.sl` binding. The native bridge owns the camera and publishes `/rover/vision/bodies`; do not run it at the same time as the ZED ROS wrapper.
- Jetson system libraries and drivers are vendor/apt-managed: **JetPack/L4T, CUDA, ZED SDK, pyzed, ROS 2 Humble, rclpy and ROS messages**. They are intentionally excluded from pip requirement files.
- `requirements-jetson.txt` contains only the extra NumPy constraint needed by the installed ZED Python binding. Do not blindly install CPU-only torch, incompatible NumPy, or unverified `pyzed`/TensorRT wheels.
- Whisper.cpp is a separately built local binary on laptop/VM (if needed). It is **not a pip requirement**; no cloud API or LLM is needed to map recognized text into a small fixed command set.

## Record real versions on target
In a VM or Jetson terminal:

```bash
bash save_deps.sh
```

This writes `deps_snapshot.txt` with installed Ubuntu packages, Python freeze and environment metadata for later reproducibility. Do not commit private environment values or credentials.
