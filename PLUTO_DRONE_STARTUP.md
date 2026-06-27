# Pluto Drone — Complete Daily Startup Guide

> Every time you want to fly, follow these steps in order.
> Two terminals needed: one for the dashboard, one for the drone node.

---

## Pre-flight Checklist

- [ ] Pluto drone is powered ON (LED blinking)
- [ ] Windows WiFi is connected to **Pluto-XXXXXX** hotspot
- [ ] Dashboard zip is already built (one-time setup done)

---

## Step 1 — Connect Windows WiFi to Pluto Hotspot

1. Click **WiFi icon** on Windows taskbar (bottom right)
2. Find and click **`Pluto-XXXXXX`** network
3. Password: **`12345678`** (default)
4. Wait for "Connected" status

**Verify in Windows PowerShell** (`Win+R` → type `powershell` → Enter):

```powershell
ping 192.168.4.1
```

Expected output — you should see replies, not timeouts:
```
Reply from 192.168.4.1: bytes=32 time=5ms TTL=128
Reply from 192.168.4.1: bytes=32 time=4ms TTL=128
```

> If all 4 packets time out — drone is not on, or WiFi didn't connect properly. Re-check.

---

## Step 2 — Fix WSL Routing (every new WSL session)

WSL2 is isolated from Windows networking by default.
This one command bridges WSL to the drone through Windows:

```bash
sudo ip route add 192.168.4.0/24 via 172.24.64.1
```

> **Note:** The error `RTNETLINK answers: File exists` is harmless — it just means the route is already there from a previous session.

Verify WSL can reach the drone:

```bash
ping -c 3 192.168.4.1
```

Expected: `0% packet loss` with response times shown.

---

## Step 3 — Terminal 1 : Start the Dashboard

Open a **new Ubuntu/WSL terminal** and run:

```bash
# Source ROS and workspace
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/dell/OneDrive/Desktop/neww/dashboard_fixed/pluto_ros2_package/install/setup.bash

# Launch dashboard
ros2 launch pluto_dashboard dashboard_launch.py
```

Then open your browser and go to:

**http://127.0.0.1:5050**

> Keep this terminal open and running in the background. Do not close it.

---

## Step 4 — Terminal 2 : Start the Drone Node (WiFi)

Open a **second Ubuntu/WSL terminal** and run:

```bash
# Source ROS and workspace
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/dell/OneDrive/Desktop/neww/dashboard_fixed/pluto_ros2_package/install/setup.bash

# Connect to drone over WiFi
ros2 run plutodrone plutonode
```

You should see the drone connecting and telemetry appearing in the dashboard at http://127.0.0.1:5050.

---

## Optional — Terminal 2 : CrazyRadio PA (USB dongle instead of WiFi)

If using the CrazyRadio PA USB dongle instead of WiFi, plug it in first, then:

```bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/dell/OneDrive/Desktop/neww/dashboard_fixed/pluto_ros2_package/install/setup.bash

ros2 launch pluto_dashboard dashboard_launch.py \
  use_radio:=true \
  radio_channel:=80 \
  radio_address:=E7E7E7E7E7
```

> For CrazyRadio, you do NOT need to connect Windows WiFi to the Pluto hotspot.

---

## Quick Reference — Both Terminals Side by Side

| | Terminal 1 | Terminal 2 |
|---|---|---|
| **Purpose** | Dashboard UI | Drone connection |
| **Command** | `ros2 launch pluto_dashboard dashboard_launch.py` | `ros2 run plutodrone plutonode` |
| **Keep open?** | Yes | Yes |
| **URL** | http://127.0.0.1:5050 | — |

---

## One-Shot Convenience Script

Save this once, run it every time instead of typing everything manually:

```bash
cat > ~/start_pluto.sh << 'SCRIPT'
#!/bin/bash

WS=/mnt/c/Users/dell/OneDrive/Desktop/neww/dashboard_fixed/pluto_ros2_package

echo "================================================"
echo "  PLUTO DRONE STARTUP"
echo "================================================"
echo ""

# Fix WSL routing to reach drone
echo "[1/3] Fixing WSL network routing..."
sudo ip route add 192.168.4.0/24 via 172.24.64.1 2>/dev/null || true

# Test drone reachability
echo "[2/3] Testing drone connection..."
if ping -c 1 -W 2 192.168.4.1 > /dev/null 2>&1; then
    echo "      ✅ Drone reachable at 192.168.4.1"
else
    echo "      ❌ Cannot reach drone at 192.168.4.1"
    echo "      → Make sure Windows WiFi is on Pluto-XXXXXX hotspot"
    echo "      → Make sure drone is powered ON"
    exit 1
fi

# Source ROS
echo "[3/3] Sourcing ROS + workspace..."
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

echo ""
echo "================================================"
echo "  Dashboard → http://127.0.0.1:5050"
echo "  Open a SECOND terminal and run:"
echo "    source /opt/ros/jazzy/setup.bash"
echo "    source $WS/install/setup.bash"
echo "    ros2 run plutodrone plutonode"
echo "================================================"
echo ""

# Launch dashboard
ros2 launch pluto_dashboard dashboard_launch.py
SCRIPT

chmod +x ~/start_pluto.sh
echo "✅ Script saved. Run with: ~/start_pluto.sh"
```

Next time just run:
```bash
~/start_pluto.sh
```

---

## Troubleshooting

### Dashboard port already in use
```bash
fuser -k 5050/tcp 2>/dev/null || true
```

### WSL lost the drone route after reboot
```bash
sudo ip route add 192.168.4.0/24 via 172.24.64.1
```

### `Package 'plutodrone' not found`
```bash
WS=/mnt/c/Users/dell/OneDrive/Desktop/neww/dashboard_fixed/pluto_ros2_package
source /opt/ros/jazzy/setup.bash
cd "$WS"
colcon build --symlink-install \
  --packages-select custom_msgs plutodrone pluto_dashboard pluto_camera_sense
source install/setup.bash
```

### `/opt/ros/humble/setup.bash: No such file or directory`
```bash
sed -i '/opt\/ros\/humble/d' ~/.bashrc
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

### Flask or opencv not found
```bash
pip3 install flask opencv-python-headless --break-system-packages
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

### Drone connects but telemetry shows zeros
The drone node connected but ROS topics aren't bridging. Check:
```bash
ros2 topic list
ros2 topic echo /drone_telemetry
```

### Takeoff drifts forward
This is fixed in `dashboard_fixed.zip`. Verify the fix is in place:
```bash
grep "belt-and-suspenders" \
  /mnt/c/Users/dell/OneDrive/Desktop/neww/dashboard_fixed/pluto_ros2_package/pluto_ros2_package/src/pluto_dashboard/pluto_dashboard/dashboard_node.py \
  && echo "✅ Takeoff fix active" \
  || echo "❌ Old version — re-copy from dashboard_fixed.zip"
```

---

## System Info (your setup)

| Item | Value |
|---|---|
| OS | Ubuntu 24.04 (WSL2) |
| ROS version | Jazzy |
| Workspace | `/mnt/c/Users/dell/OneDrive/Desktop/neww/dashboard_fixed/pluto_ros2_package` |
| Dashboard URL | http://127.0.0.1:5050 |
| Drone AP IP | 192.168.4.1 |
| WSL gateway | 172.24.64.1 |
| Drone WiFi | Pluto-XXXXXX (password: 12345678) |
