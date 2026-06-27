# PlutoDrone ROS 2 Dashboard

A full ROS 2 software stack for real-hardware drone operation — connecting to a live **PlutoDrone** over WiFi, streaming H.264 video at 25 Hz, overlaying MediaPipe hand detection, and serving a real-time telemetry dashboard at `localhost:5050`.

---

## System Architecture

```
PlutoDrone (192.168.0.1)
    │
    ├── port 9060 ──► plutonode ──► ROS 2 /pluto/* topics  (flight control)
    │
    └── port 7065 ──► plutocam_publisher
                           │
                           ▼
                      FFmpeg (H264 → BGR24)
                           │
                           ▼
                      MediaPipe (hand overlay / ArUco mode)
                           │
                           ▼
                      /plutocamera/image_raw  (ROS 2 topic @ 25 Hz)
                           │
                           ▼
                      dashboard_node ──► http://localhost:5050
```

---

## Features

- **Live flight control** — ROS 2 node (`plutonode`) connects to the drone over WiFi on port 9060
- **Real-time video stream** — H.264 camera stream decoded via FFmpeg at 25 Hz and published as a ROS 2 image topic
- **Computer vision overlay** — MediaPipe hand detection and ArUco marker tracking on the live feed
- **Web dashboard** — Flask-served UI at `localhost:5050` with live camera feed and telemetry
- **RViz integration** — 3D visualization of drone pose and sensor data
- **Non-blocking vision pipeline** — dedicated worker thread prevents camera feed freezes during CV processing

---

## Tech Stack

| Component | Technology |
|---|---|
| Middleware | ROS 2 Jazzy |
| Flight control | PlutoDrone SDK (`plutonode`) |
| Video decoding | FFmpeg (H.264 → BGR24) |
| Computer vision | OpenCV, MediaPipe |
| Web dashboard | Flask, HTML/CSS/JS |
| Visualization | RViz |
| OS | Ubuntu 24 (WSL2 on Windows 11) |
| Language | Python 3.12 |

---

## Port Reference

| Port | Used By | Purpose |
|---|---|---|
| `9060` | `plutonode` | Flight control commands |
| `7065` | `plutocam_publisher` | H.264 video stream |
| `8065` | `plutonode` | Command acknowledgements |
| `5050` | `dashboard_node` | Web UI (Flask) |

---

## Repository Structure

```
documentation/
├── dashboard_fixed/              # Fixed dashboard with non-blocking vision worker
│   └── pluto_ros2_package/
│       └── src/
│           ├── pluto_dashboard/  # Flask dashboard node + HTML templates
│           └── pluto_camera_sense/ # Camera publisher, FFmpeg decoder, defaults
├── dashboard_rviz/               # RViz-integrated variant of the dashboard
│   └── pluto_ros2_package/
├── pluto_fixed/                  # Core ROS 2 package sources
│   └── pluto_ros2_package/
├── DASHBOARD_SETUP.md            # One-time dashboard setup guide
├── PLUTO_DRONE_STARTUP.md        # Drone connection and flight control startup
├── PLUTO_CAMERA_STARTUP.md       # Camera publisher startup guide
├── PLUTO_RVIZ_STARTUP.md         # RViz visualization setup
├── PLUTO_RUN_GUIDE.md            # Full end-to-end run guide (start here)
├── dashboard_rviz.zip            # Packaged RViz dashboard
└── index.html                    # Dashboard web entry point
```

---

## Prerequisites

- **OS:** Windows 11 + WSL2 (Ubuntu 24) or native Ubuntu 24
- **ROS 2:** Jazzy
- **Python:** 3.12
- **Hardware:** PlutoDrone connected via WiFi at `192.168.0.1`

---

## One-Time Setup

### 1. Fix NumPy / OpenCV compatibility

```bash
pip install "numpy<2" --break-system-packages
pip install "opencv-python-headless==4.8.1.78" --break-system-packages
pip install mediapipe --break-system-packages
pip install av --break-system-packages
```

### 2. Build the ROS 2 packages

```bash
source /opt/ros/jazzy/setup.bash
cd dashboard_fixed/pluto_ros2_package
colcon build --symlink-install --packages-select pluto_dashboard
source install/setup.bash
```

### 3. Verify

```bash
python3 -c "import cv2; import numpy; from cv_bridge import CvBridge; \
  print('ALL OK — numpy:', numpy.__version__, 'cv2:', cv2.__version__)"
```

---

## Running the System

Connect your PC to the PlutoDrone WiFi network and verify:

```bash
ping 192.168.0.1 -c 3   # Expect 0% packet loss, ~3–5 ms latency
```

Then open **three terminals** in WSL2:

### Terminal 1 — Flight Control Node

```bash
source /opt/ros/jazzy/setup.bash
source dashboard_rviz/pluto_ros2_package/install/setup.bash
ros2 run plutodrone plutonode
# Wait for: [plutonode] connected to 192.168.0.1:9060
```

### Terminal 2 — Camera Publisher

```bash
source /opt/ros/jazzy/setup.bash
source dashboard_fixed/pluto_ros2_package/install/setup.bash
plutocam_publisher --low-def
# Wait for: Published 30 frames ... Published 60 frames ...
```

### Terminal 3 — Web Dashboard

```bash
source /opt/ros/jazzy/setup.bash
source dashboard_fixed/pluto_ros2_package/install/setup.bash
ros2 launch pluto_dashboard dashboard_launch.py
# Open: http://localhost:5050
```

### Verify Camera Feed (optional Terminal 4)

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic hz /plutocamera/image_raw --window 20
# Expected: average rate: 25.0 Hz
```

---

## Key File Locations

| File | Path |
|---|---|
| Camera publisher | `src/pluto_camera_sense/pluto_camera_sense/plutocam_publisher.py` |
| Dashboard node | `src/pluto_dashboard/pluto_dashboard/dashboard_node.py` |
| Dashboard HTML | `src/pluto_dashboard/pluto_dashboard/templates/index.html` |
| Drone defaults (IP/ports) | `src/pluto_camera_sense/pluto_camera_sense/defaults.py` |
| LWDrone stream reader | `src/pluto_camera_sense/pluto_camera_sense/lwdrone.py` |

---

## Rebuild After Code Changes

```bash
cd dashboard_fixed/pluto_ros2_package

colcon build --packages-select pluto_camera_sense
colcon build --packages-select pluto_dashboard

# Always re-source after rebuild
source install/setup.bash
```

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `ping 192.168.0.1` fails | Reconnect to drone WiFi |
| `plutocam_publisher` times out | Kill `plutonode` first; check port: `ss -tn \| grep 7065` |
| `numpy` error on launch | `pip install "numpy<2" --break-system-packages` |
| `opencv` conflict with numpy | `pip install "opencv-python-headless==4.8.1.78" --break-system-packages` |
| `ros2 topic hz` shows 0 | Check Terminal 2 — `plutocam_publisher` must show "Published X frames" |
| Dashboard shows no feed | Confirm: `ros2 topic list \| grep pluto` shows `/plutocamera/image_raw` |
| Camera feed freezes in hand/ArUco mode | Deploy fixed `dashboard_node.py` with non-blocking vision worker |

---

## Documentation Index

| Guide | Description |
|---|---|
| [`PLUTO_RUN_GUIDE.md`](./PLUTO_RUN_GUIDE.md) | Full end-to-end run guide — start here |
| [`PLUTO_DRONE_STARTUP.md`](./PLUTO_DRONE_STARTUP.md) | Drone connection and flight control |
| [`PLUTO_CAMERA_STARTUP.md`](./PLUTO_CAMERA_STARTUP.md) | Camera publisher setup |
| [`PLUTO_RVIZ_STARTUP.md`](./PLUTO_RVIZ_STARTUP.md) | RViz visualization setup |
| [`DASHBOARD_SETUP.md`](./DASHBOARD_SETUP.md) | One-time dashboard installation |

---

## Author

**Krish Jain** — Department of Electrical Engineering, IIT Jodhpur
GitHub: [@Krish-Jain-IITJ](https://github.com/Krish-Jain-IITJ)

---

## License

This project is currently unlicensed. Please contact the author for usage permissions.
