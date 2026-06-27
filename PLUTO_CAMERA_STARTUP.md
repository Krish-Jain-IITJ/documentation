# Pluto Drone — Camera Feed Startup Guide

> Follow these steps every time you want to see the camera feed.
> **3 terminals needed.**

---

## Pre-flight Checklist

- [ ] Pluto drone is powered ON (LED blinking)
- [ ] PlutoCam module attached to drone
- [ ] Windows WiFi connected to **WIFI-1080p-XXXXXX** (password: `12345678`)
- [ ] Workspace built: `dashboard_fixed`

---

## Step 1 — Fix WSL Routing (every new WSL session)

```bash
GW=$(ip route show | grep default | awk '{print $3}')
sudo ip route del 192.168.0.0/24 2>/dev/null || true
sudo ip route add 192.168.0.0/24 via $GW
ping -c 3 192.168.0.1
```

Expected: `0% packet loss`

> If timeout — Windows WiFi is not on WIFI-1080p-* hotspot. Re-check.

---

## Step 2 — Terminal 1 : Dashboard

```bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/dell/OneDrive/Desktop/neww/dashboard_fixed/pluto_ros2_package/install/setup.bash
ros2 launch pluto_dashboard dashboard_launch.py
```

Expected: Flask running at `http://127.0.0.1:5050`

> Keep this terminal open. Do not close it.

---

## Step 3 — Terminal 2 : Drone Node

Open a **new WSL terminal**:

```bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/dell/OneDrive/Desktop/neww/dashboard_rviz/pluto_ros2_package/install/setup.bash
ros2 run plutodrone plutonode
```

Expected: `Connected to Pluto at 192.168.0.1:9060`

> If you see `Cannot connect` — WiFi is not on WIFI-1080p-* hotspot. Go back to Step 1.

---

## Step 4 — Terminal 3 : Camera Node

Open a **new WSL terminal**:

```bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/dell/OneDrive/Desktop/neww/dashboard_fixed/pluto_ros2_package/install/setup.bash
/mnt/c/Users/dell/OneDrive/Desktop/neww/dashboard_fixed/pluto_ros2_package/install/pluto_camera_sense/bin/plutocam_publisher
```

Expected:
```
Connecting to PlutoCamera at 192.168.0.1...
Connected to PlutoCamera at 192.168.0.1
Starting 720p video stream... (Press Ctrl+C to stop)
```

> Keep this terminal open. Do not close it.

---

## Step 5 — Verify Feed is Flowing (optional check)

In any terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic hz /plutocamera/image_raw --window 20
```

Expected: `average rate: 10.000 — 25.000 Hz`

> Press Ctrl+C after confirming.

---

## Step 6 — Open Feed in Browser

Open **Windows browser** and go to:

| URL | What it shows |
|---|---|
| `http://127.0.0.1:5050` | Full dashboard + camera panel |
| `http://127.0.0.1:5050/api/camera/stream` | Raw live video feed only |
| `http://127.0.0.1:5050/api/camera/snapshot` | Single snapshot image |
| `http://127.0.0.1:5050/api/camera/status` | JSON status — streaming true/false |

---

## All 3 Terminals at a Glance

| Terminal | Workspace | Command | What it does |
|---|---|---|---|
| T1 | `dashboard_fixed` | `ros2 launch pluto_dashboard dashboard_launch.py` | Web dashboard at :5050 |
| T2 | `dashboard_rviz` | `ros2 run plutodrone plutonode` | WiFi drone + flight control |
| T3 | `dashboard_fixed` | run `plutocam_publisher` binary | Camera feed → ROS topic → web |

---

## Troubleshooting

### `No executable found` for plutocam_publisher
Package not built. Run:
```bash
cd /mnt/c/Users/dell/OneDrive/Desktop/neww/dashboard_fixed/pluto_ros2_package
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select pluto_camera_sense
source install/setup.bash
```

### Camera node crashes with NumPy error
```bash
pip install "numpy<2" --break-system-packages
```

### `ffmpeg not found` error
```bash
sudo apt install ffmpeg -y
```

### `streaming: false` in camera status
Camera node is not running or not receiving frames. Check Terminal 3.

### Feed is very slow (< 5 Hz)
Normal on first connect — FFmpeg needs a few seconds to buffer.
If stays slow after 30 seconds, restart Terminal 3.

### WSL lost camera route after reboot
```bash
GW=$(ip route show | grep default | awk '{print $3}')
sudo ip route add 192.168.0.0/24 via $GW
```

### Dashboard port 5050 already in use
```bash
fuser -k 5050/tcp 2>/dev/null || true
```

---

## System Info

| Item | Value |
|---|---|
| OS | Ubuntu 24.04 (WSL2) |
| ROS version | Jazzy |
| Camera workspace | `dashboard_fixed/pluto_ros2_package` |
| Drone workspace | `dashboard_rviz/pluto_ros2_package` |
| Dashboard URL | http://127.0.0.1:5050 |
| Camera AP IP | 192.168.0.1 |
| Camera cmd port | 8065 |
| Camera stream port | 7065 |
| Drone relay port | 9060 |
| Camera resolution | 720p (1280×720) |
| Target frame rate | 20–25 Hz |
| Pluto WiFi | WIFI-1080p-XXXXXX (password: 12345678) |
