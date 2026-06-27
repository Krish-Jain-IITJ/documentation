# Pluto Drone — Complete Daily Startup Guide (Dashboard + RViz)

> Every time you want to fly and visualize, follow these steps in order.
> **4 terminals needed** — one for each component.

---

## Pre-flight Checklist

- [ ] Pluto drone is powered ON (LED blinking)
- [ ] Windows WiFi connected to **Pluto-XXXXXX** hotspot (password: `12345678`)
- [ ] Both workspaces built: `dashboard_fixed` and `dashboard_rviz`

---

## Step 1 — Connect Windows WiFi to Pluto Hotspot

1. Click **WiFi icon** on Windows taskbar (bottom right)
2. Find and connect to **`Pluto-XXXXXX`** network
3. Password: **`12345678`**

**Verify in Windows PowerShell** (`Win+R` → `powershell` → Enter):

```powershell
ping 192.168.4.1
```

Expected — replies, not timeouts:
```
Reply from 192.168.4.1: bytes=32 time=5ms TTL=128
Reply from 192.168.4.1: bytes=32 time=4ms TTL=128
```

> If all packets time out — drone is not ON or WiFi didn't connect. Re-check.

---

## Step 2 — Fix WSL Routing (every new WSL session)

Open a WSL terminal and run:

```bash
sudo ip route add 192.168.4.0/24 via 172.24.64.1 2>/dev/null || true
```

Verify WSL can reach the drone:

```bash
ping -c 3 192.168.4.1
```

Expected: `0% packet loss`

> The error `RTNETLINK answers: File exists` is harmless — route already added.

---

## Step 3 — Terminal 1 : Dashboard (optional but recommended)

```bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/dell/OneDrive/Desktop/neww/dashboard_fixed/pluto_ros2_package/install/setup.bash

ros2 launch pluto_dashboard dashboard_launch.py
```

Open browser → **http://127.0.0.1:5050**

> Keep this terminal open. Do not close it.

---

## Step 4 — Terminal 2 : Drone Node (WiFi connection)

Open a **new WSL terminal**:

```bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/dell/OneDrive/Desktop/neww/dashboard_rviz/pluto_ros2_package/install/setup.bash

ros2 run plutodrone plutonode
```

Expected output — drone connects and starts streaming data:
```
[plutonode] Connected to Pluto at 192.168.4.1:23
```

> If you see "Cannot connect" — WiFi is not on Pluto hotspot. Go back to Step 1.

---

## Step 5 — Terminal 3 : RViz Visualization

Open a **new WSL terminal**:

```bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/dell/OneDrive/Desktop/neww/dashboard_rviz/pluto_ros2_package/install/setup.bash

ros2 launch pluto_dashboard rviz_launch.py
```

RViz window opens with the Pluto 3D drone model on a grid.

> **Windows 11** — RViz opens automatically via WSLg.
> **Windows 10** — Run `export DISPLAY=:0` before this command.

---

## Step 6 — Terminal 4 : TF Broadcaster (brings drone model to life)

Open a **new WSL terminal**:

```bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/dell/OneDrive/Desktop/neww/dashboard_rviz/pluto_ros2_package/install/setup.bash

ros2 run pluto_dashboard tf_broadcaster
```

Expected output:
```
[INFO] URDF loaded from .../urdf/pluto.urdf
[INFO] PlutoTFBroadcaster started. Waiting for /pluto/imu and /pluto/altitude ...
[INFO] IMU data received — attitude broadcasting active.
[INFO] Altitude data received — Z axis broadcasting active.
```

**The moment these two INFO lines appear — look at RViz.**
The drone model will tilt in real time as you physically move the drone.

---

## All 4 Terminals at a Glance

| Terminal | Workspace | Command | What it does |
|---|---|---|---|
| T1 | `dashboard_fixed` | `ros2 launch pluto_dashboard dashboard_launch.py` | Web dashboard at :5050 |
| T2 | `dashboard_rviz` | `ros2 run plutodrone plutonode` | WiFi drone connection |
| T3 | `dashboard_rviz` | `ros2 launch pluto_dashboard rviz_launch.py` | Opens RViz window |
| T4 | `dashboard_rviz` | `ros2 run pluto_dashboard tf_broadcaster` | Live 3D attitude in RViz |

---

## One-Shot Convenience Script

Save once, use every time:

```bash
cat > ~/start_rviz.sh << 'SCRIPT'
#!/bin/bash

WS_DASH=/mnt/c/Users/dell/OneDrive/Desktop/neww/dashboard_fixed/pluto_ros2_package
WS_RVIZ=/mnt/c/Users/dell/OneDrive/Desktop/neww/dashboard_rviz/pluto_ros2_package

echo "================================================"
echo "  PLUTO DRONE — RViz STARTUP"
echo "================================================"

# Fix WSL routing
echo "[1/3] Fixing WSL routing..."
sudo ip route add 192.168.4.0/24 via 172.24.64.1 2>/dev/null || true

# Test drone reachability
echo "[2/3] Testing drone connection..."
if ping -c 1 -W 2 192.168.4.1 > /dev/null 2>&1; then
    echo "      ✅ Drone reachable at 192.168.4.1"
else
    echo "      ❌ Cannot reach drone — connect WiFi to Pluto-XXXXXX first"
    exit 1
fi

# Source RViz workspace
echo "[3/3] Sourcing workspace..."
source /opt/ros/jazzy/setup.bash
source "$WS_RVIZ/install/setup.bash"

echo ""
echo "================================================"
echo "  Now open 3 more terminals and run:"
echo ""
echo "  T2: ros2 run plutodrone plutonode"
echo "  T3: ros2 launch pluto_dashboard rviz_launch.py"
echo "  T4: ros2 run pluto_dashboard tf_broadcaster"
echo "================================================"
echo ""

# This terminal runs the dashboard
source "$WS_DASH/install/setup.bash" 2>/dev/null || true
ros2 launch pluto_dashboard dashboard_launch.py
SCRIPT

chmod +x ~/start_rviz.sh
echo "✅ Script saved. Run with: ~/start_rviz.sh"
```

---

## Verify Data is Flowing (optional check)

In any terminal after Step 4:

```bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/dell/OneDrive/Desktop/neww/dashboard_rviz/pluto_ros2_package/install/setup.bash

# IMU should show ~150Hz
ros2 topic hz /pluto/imu

# TF should show odom → base_link transform
ros2 run tf2_tools view_frames
```

---

## Troubleshooting

### RViz drone model is completely still (not moving)
TF broadcaster is not running or has QoS mismatch. Check Terminal 4:
```bash
# Should show these two INFO lines:
# [INFO] IMU data received — attitude broadcasting active.
# [INFO] Altitude data received — Z axis broadcasting active.

# If you see QoS RELIABILITY warnings, the fix is already applied.
# Just restart tf_broadcaster:
ros2 run pluto_dashboard tf_broadcaster
```

### QoS incompatible warnings in tf_broadcaster
Already fixed in `dashboard_rviz` — subscriptions use `BEST_EFFORT` to match
the drone node's publisher. If it reappears, verify the fix is in the file:
```bash
grep "BEST_EFFORT" \
  /mnt/c/Users/dell/OneDrive/Desktop/neww/dashboard_rviz/pluto_ros2_package/pluto_ros2_package/src/pluto_dashboard/pluto_dashboard/pluto_tf_broadcaster.py \
  && echo "✅ QoS fix present" \
  || echo "❌ Fix missing — re-apply manually"
```

### RViz shows blank / no robot model
`robot_state_publisher` didn't get the URDF. Restart RViz launch:
```bash
# Ctrl+C in Terminal 3, then rerun:
ros2 launch pluto_dashboard rviz_launch.py
```

### `Package 'plutodrone' not found`
```bash
WS=/mnt/c/Users/dell/OneDrive/Desktop/neww/dashboard_rviz/pluto_ros2_package
source /opt/ros/jazzy/setup.bash
cd "$WS"
colcon build --symlink-install \
  --packages-select custom_msgs plutodrone pluto_dashboard pluto_camera_sense
source install/setup.bash
```

### WSL lost drone route after reboot
```bash
sudo ip route add 192.168.4.0/24 via 172.24.64.1
```

### RViz doesn't open (no display)
```bash
export DISPLAY=:0
ros2 launch pluto_dashboard rviz_launch.py
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
| Dashboard workspace | `dashboard_fixed/pluto_ros2_package` |
| RViz workspace | `dashboard_rviz/pluto_ros2_package` |
| Dashboard URL | http://127.0.0.1:5050 |
| Drone AP IP | 192.168.4.1 |
| WSL gateway | 172.24.64.1 |
| Pluto WiFi | Pluto-XXXXXX (password: 12345678) |
| IMU publish rate | ~152 Hz |
| TF broadcast rate | 30 Hz |
