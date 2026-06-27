# Pluto Dashboard — Complete WSL Setup & Run Guide

> **All commands below run inside WSL (Ubuntu).** Paste each block into your WSL terminal in order.
> The dashboard will be available at **http://127.0.0.1:5050** once step 5 is complete.

---

## Prerequisites checklist (do once)

| Requirement | How to verify |
|---|---|
| WSL2 with Ubuntu 22.04 | `lsb_release -a` → Ubuntu 22.04 |
| ROS 2 Humble installed | `ros2 --version` → `humble` |
| `colcon` build tool | `colcon --version` |
| Python 3.10 | `python3 --version` → 3.10.x |

---

## Step 1 — Install system dependencies (one-time)

```bash
sudo apt-get update -y

# ROS 2 build + Python tools
sudo apt-get install -y \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-cv-bridge \
  ros-humble-sensor-msgs \
  ros-humble-std-msgs \
  ros-humble-rclpy

# Python packages the dashboard needs
pip3 install flask opencv-python-headless

# (Optional) CrazyRadio PA support — only needed if using USB dongle
# pip3 install cflib
```

---

## Step 2 — Place the fixed source files

The workspace root is already on your Windows drive, accessible from WSL at:

```
/mnt/c/Users/dell/OneDrive/Desktop/dashboard/pluto_ros2_package
```

Download `dashboard_fixed.zip` (provided), then run:

```bash
# Set workspace path variable (used throughout this guide)
export WS=/mnt/c/Users/dell/OneDrive/Desktop/dashboard/pluto_ros2_package

# Unzip fixed files (adjust path to wherever you saved the zip)
ZIPFILE="$HOME/Downloads/dashboard_fixed.zip"          # ← change if saved elsewhere
unzip -o "$ZIPFILE" -d /tmp/dashboard_fix

# Overwrite only the three changed packages — leaves everything else intact
SRC="$WS/pluto_ros2_package/src"

cp -r /tmp/dashboard_fix/dashboard_fixed/pluto_ros2_package/pluto_ros2_package/src/pluto_dashboard \
      "$SRC/"

cp -r /tmp/dashboard_fix/dashboard_fixed/pluto_ros2_package/pluto_ros2_package/src/pluto_camera_sense \
      "$SRC/"

cp -r /tmp/dashboard_fix/dashboard_fixed/pluto_ros2_package/pluto_ros2_package/src/custom_msgs \
      "$SRC/"

echo "✅ Source files copied"
```

---

## Step 3 — Initialize rosdep (one-time, skip if already done)

```bash
source /opt/ros/humble/setup.bash

sudo rosdep init 2>/dev/null || true   # ignore "already initialized" error
rosdep update

# Install all ROS deps declared in package.xml files
cd "$WS"
rosdep install --from-paths pluto_ros2_package/src --ignore-src -r -y

echo "✅ rosdep done"
```

---

## Step 4 — Build the workspace

```bash
export WS=/mnt/c/Users/dell/OneDrive/Desktop/dashboard/pluto_ros2_package

source /opt/ros/humble/setup.bash

cd "$WS"

# Clean only the three affected packages (faster than full clean)
rm -rf build/custom_msgs build/pluto_dashboard build/pluto_camera_sense
rm -rf install/custom_msgs install/pluto_dashboard install/pluto_camera_sense

# Build (symlink-install means edits to .py files take effect without rebuilding)
colcon build \
  --symlink-install \
  --packages-select custom_msgs pluto_dashboard pluto_camera_sense \
  --event-handlers console_cohesion+

echo "✅ Build complete — exit code: $?"
```

> **If you see `custom_msgs` errors** like `rosidl_generator` missing, run a full build first:
> ```bash
> colcon build --symlink-install
> ```

---

## Step 5 — Run the dashboard

### Option A — WiFi connection (Pluto AP or WIFI-1080p-*)

```bash
export WS=/mnt/c/Users/dell/OneDrive/Desktop/dashboard/pluto_ros2_package

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# Start dashboard only (drone node started from the UI)
ros2 launch pluto_dashboard dashboard_launch.py
```

### Option B — WiFi + PlutoCam

```bash
export WS=/mnt/c/Users/dell/OneDrive/Desktop/dashboard/pluto_ros2_package

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

ros2 launch pluto_dashboard dashboard_launch.py use_camera:=true cam_ip:=192.168.0.1
```

### Option C — CrazyRadio PA USB dongle (no WiFi needed)

```bash
# Install cflib once
pip3 install cflib

export WS=/mnt/c/Users/dell/OneDrive/Desktop/dashboard/pluto_ros2_package

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# Plug CrazyRadio PA USB dongle before running
ros2 launch pluto_dashboard dashboard_launch.py \
  use_radio:=true \
  radio_channel:=80 \
  radio_address:=E7E7E7E7E7
```

> After launch, open **http://127.0.0.1:5050** in your browser.

---

## Step 6 — Start the drone node (WiFi modes)

In a **second WSL terminal** (after step 5 is running):

```bash
export WS=/mnt/c/Users/dell/OneDrive/Desktop/dashboard/pluto_ros2_package

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# Connect your laptop to the Pluto WiFi AP first, then:
ros2 run plutodrone plutonode
```

Or use the **Services → Start** button directly in the dashboard UI — it launches `start_drone.sh` for you.

---

## One-shot convenience script (copy-paste the whole block)

Save this as `~/run_dashboard.sh` and run it anytime:

```bash
cat > ~/run_dashboard.sh << 'SCRIPT'
#!/bin/bash
set -e

WS=/mnt/c/Users/dell/OneDrive/Desktop/dashboard/pluto_ros2_package
ROS_SETUP=/opt/ros/humble/setup.bash

# ── Colour helpers ─────────────────────────────────────
GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
info()  { echo -e "${GREEN}[dashboard]${NC} $*"; }
warn()  { echo -e "${YELLOW}[dashboard]${NC} $*"; }

# ── Source ROS ────────────────────────────────────────
source "$ROS_SETUP"
source "$WS/install/setup.bash"

# ── Parse args ────────────────────────────────────────
USE_CAMERA=false
USE_RADIO=false
CAM_IP=192.168.0.1
RADIO_CH=80
RADIO_ADDR=E7E7E7E7E7

while [[ $# -gt 0 ]]; do
  case $1 in
    --camera)  USE_CAMERA=true ;;
    --radio)   USE_RADIO=true  ;;
    --cam-ip)  CAM_IP=$2;   shift ;;
    --channel) RADIO_CH=$2; shift ;;
    --address) RADIO_ADDR=$2; shift ;;
    *) warn "Unknown arg: $1" ;;
  esac
  shift
done

# ── Launch ────────────────────────────────────────────
ARGS="use_camera:=$USE_CAMERA use_radio:=$USE_RADIO"
ARGS="$ARGS cam_ip:=$CAM_IP"
ARGS="$ARGS radio_channel:=$RADIO_CH radio_address:=$RADIO_ADDR"

info "Launching dashboard (http://127.0.0.1:5050)"
info "  camera=$USE_CAMERA  radio=$USE_RADIO"
ros2 launch pluto_dashboard dashboard_launch.py $ARGS
SCRIPT

chmod +x ~/run_dashboard.sh
echo "✅ Script saved. Run with:"
echo "   ~/run_dashboard.sh                          # WiFi only"
echo "   ~/run_dashboard.sh --camera                 # WiFi + PlutoCam"
echo "   ~/run_dashboard.sh --radio                  # CrazyRadio PA"
echo "   ~/run_dashboard.sh --radio --channel 40     # Custom channel"
```

---

## Troubleshooting

### Dashboard port already in use
```bash
# Kill whatever is on 5050
fuser -k 5050/tcp 2>/dev/null || true
```

### `custom_msgs` not found after build
```bash
source /opt/ros/humble/setup.bash
source /mnt/c/Users/dell/OneDrive/Desktop/dashboard/pluto_ros2_package/install/setup.bash
# Then retry ros2 launch
```

### `ModuleNotFoundError: No module named 'flask'`
```bash
pip3 install flask
```

### `cv_bridge` import error (camera tab shows warning, rest works fine)
```bash
sudo apt-get install -y ros-humble-cv-bridge python3-opencv
```

### CrazyRadio not detected on WSL2
```bash
# On Windows PowerShell (as Administrator) — pass USB through to WSL2
winget install usbipd
usbipd list                          # find the CrazyRadio device
usbipd bind --busid <BUSID>          # e.g. 1-3
usbipd attach --wsl --busid <BUSID>

# Back in WSL — verify it arrived
lsusb | grep -i crazy
```

### Drone drifts forward on takeoff
This was a bug in the original `_delayed_takeoff()` — it's fixed in `dashboard_fixed.zip`. If you still see it, confirm the new version is active:
```bash
grep "belt-and-suspenders" \
  /mnt/c/Users/dell/OneDrive/Desktop/dashboard/pluto_ros2_package/pluto_ros2_package/src/pluto_dashboard/pluto_dashboard/dashboard_node.py \
  && echo "✅ Fixed version is in place" \
  || echo "❌ Old version — re-copy from dashboard_fixed.zip"
```

### Full workspace clean rebuild (nuclear option)
```bash
cd /mnt/c/Users/dell/OneDrive/Desktop/dashboard/pluto_ros2_package
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Port reference

| URL | What it does |
|---|---|
| http://127.0.0.1:5050/ | Dashboard UI |
| http://127.0.0.1:5050/api/telemetry | Live telemetry JSON |
| http://127.0.0.1:5050/api/network/probe | Connection mode detection |
| http://127.0.0.1:5050/api/radio/connect | POST — start CrazyRadio bridge |
| http://127.0.0.1:5050/api/camera/stream | MJPEG video stream |
| http://127.0.0.1:5050/api/log/list | Flight log CSV list |
