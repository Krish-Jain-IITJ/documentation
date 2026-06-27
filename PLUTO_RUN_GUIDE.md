# PlutoDrone Dashboard — Run Guide

## System Requirements
- OS: Windows 11 + WSL2 (Ubuntu 24)
- ROS2: Jazzy
- Python: 3.12
- Drone: PlutoDrone connected via WiFi at `192.168.0.1`

---

## One-Time Setup (do this once after fresh install)

### Fix NumPy / OpenCV compatibility
```bash
pip install "numpy<2" --break-system-packages
pip install "opencv-python-headless==4.8.1.78" --break-system-packages
pip install mediapipe --break-system-packages
pip install av --break-system-packages
```

### Verify everything works
```bash
python3 -c "import cv2; import numpy; from cv_bridge import CvBridge; print('ALL OK — numpy:', numpy.__version__, 'cv2:', cv2.__version__)"
```

---
source /opt/ros/jazzy/setup.bash
cd /mnt/c/Users/dell/OneDrive/Desktop/neww/dashboard_fixed/pluto_ros2_package
colcon build --symlink-install --packages-select pluto_dashboard
source install/setup.bash

## Every Time You Run

### Step 1 — Connect to drone WiFi
Connect your PC to the PlutoDrone WiFi network.
Verify connection:
```bash
ping 192.168.0.1 -c 3
```
Expected: 0% packet loss, ~3-5ms latency.

---

### Step 2 — Terminal 1: Start Drone Node (flight control)
```bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/dell/OneDrive/Desktop/neww/dashboard_rviz/pluto_ros2_package/install/setup.bash
ros2 run plutodrone plutonode
```
Wait for:
```
[plutonode] connected to 192.168.0.1:9060
```

---

### Step 3 — Terminal 2: Start Camera Publisher
```bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/dell/OneDrive/Desktop/neww/dashboard_fixed/pluto_ros2_package/install/setup.bash
plutocam_publisher --low-def
```
Wait for:
```
Connected to PlutoCamera at 192.168.0.1
Published 30 frames
Published 60 frames ...
```
> **Note:** `plutonode` uses port `9060` (flight control).  
> `plutocam_publisher` uses port `7065` (video stream).  
> They do NOT conflict — both can run simultaneously.

---

### Step 4 — Terminal 3: Start Dashboard
```bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/dell/OneDrive/Desktop/neww/dashboard_fixed/pluto_ros2_package/install/setup.bash
ros2 launch pluto_dashboard dashboard_launch.py
```
Wait for:
```
pluto_dashboard ready — http://localhost:5050
```
Open browser: **http://localhost:5050**

---

## Verify Camera Feed is Working

In a 4th terminal:
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic hz /plutocamera/image_raw --window 20
```
Expected output:
```
average rate: 25.0 Hz
```

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `ping 192.168.0.1` fails | Reconnect to drone WiFi |
| `plutocam_publisher` times out | Kill plutonode first, check if port 7065 is free: `ss -tn \| grep 7065` |
| `numpy` error on launch | Run: `pip install "numpy<2" --break-system-packages` |
| `opencv` conflict with numpy | Run: `pip install "opencv-python-headless==4.8.1.78" --break-system-packages` |
| `ros2 topic hz` shows 0 | Check Terminal 2 — plutocam_publisher must show "Published X frames" |
| Dashboard shows no feed | Confirm `ros2 topic list \| grep pluto` shows `/plutocamera/image_raw` |
| `[repo root: /home]` in logs | Deploy fixed `dashboard_node.py` and rebuild `pluto_dashboard` |
| Camera feed freezes with hands/aruco mode | Deploy fixed `dashboard_node.py` with non-blocking vision worker |

---

## Rebuild After Code Changes

```bash
cd /mnt/c/Users/dell/OneDrive/Desktop/neww/dashboard_fixed/pluto_ros2_package

# Rebuild specific package (faster)
colcon build --packages-select pluto_camera_sense
colcon build --packages-select pluto_dashboard

# Or rebuild everything
colcon build

# Always re-source after rebuild
source install/setup.bash
```

---

## Key File Locations

| File | Path |
|---|---|
| Camera publisher source | `src/pluto_camera_sense/pluto_camera_sense/plutocam_publisher.py` |
| Dashboard source | `src/pluto_dashboard/pluto_dashboard/dashboard_node.py` |
| Dashboard HTML | `src/pluto_dashboard/pluto_dashboard/templates/index.html` |
| Drone defaults (IP/ports) | `src/pluto_camera_sense/pluto_camera_sense/defaults.py` |
| LWDrone stream reader | `src/pluto_camera_sense/pluto_camera_sense/lwdrone.py` |

---

## Port Reference

| Port | Used By | Purpose |
|---|---|---|
| `9060` | plutonode | Flight control commands |
| `7065` | plutocam_publisher | H264 video stream |
| `8065` | plutonode | Command acknowledgements |
| `5050` | dashboard | Web UI (Flask) |

---

## Architecture Summary

```
PlutoDrone (192.168.0.1)
    │
    ├── port 9060 ──► plutonode ──► ROS2 /pluto/* topics
    │
    └── port 7065 ──► plutocam_publisher
                           │
                           ▼
                      FFmpeg (H264 → BGR24)
                           │
                           ▼
                      MediaPipe (hand overlay)
                           │
                           ▼
                      /plutocamera/image_raw  (ROS2 topic, 25 Hz)
                           │
                           ▼
                      dashboard_node ──► http://localhost:5050
```
