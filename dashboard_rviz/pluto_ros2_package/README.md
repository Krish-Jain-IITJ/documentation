# Pluto ROS 2 — dashboard integration fork

Fork of [DronaAviation/pluto_ros2_package](https://github.com/DronaAviation/pluto_ros2_package) that turns the Pluto drone into a **single-click flight stack**: open a browser, click Start, fly. Runs natively on macOS (Apple Silicon) via RoboStack; no Docker, no second laptop.

**Added on top of upstream:**
- **`pluto_dashboard`** — Flask + rclpy web UI at `http://localhost:5050`. Arm / takeoff / land, sticks, keyboard / gamepad control, live telemetry, flight logging, camera preview, process manager.
- **`pluto_vision`** — ArUco marker follower node (disabled by default).
- **Fixed telemetry parsing** in `plutodrone` (signed int16/32, Magis v2 INA219 BMS, correct MSP_ANALOG byte layout).
- **Camera-aware connection**: auto-detects whether you're on the drone's direct AP or the camera module's AP (which bridges MSP through to the flight controller).
- **Standard `sensor_msgs` topics** — `/pluto/imu`, `/pluto/battery`, `/pluto/altitude` — so rviz, foxglove, and `rosbag2` work out of the box.
- **MSP firmware commands** exposed as ROS topics (`/pluto/calibrate_acc`, `/calibrate_mag`, `/eeprom_write`, `/motor_test`, `/acc_trim`, `/set_pid`).

For the companion camera patch (1080p framebuffer fix) see the **`fix-1080p-framebuffer`** branch on [techsavvyomi/pluto_cam_ros2](https://github.com/techsavvyomi/pluto_cam_ros2).

---

## Quick start (macOS Apple Silicon)

```bash
# 1. One-time: install RoboStack ROS 2 Humble into a conda env called "ros2"
./bootstrap_ros.sh

# 2. Clone this fork + the camera fork side-by-side
mkdir -p ~/pluto && cd ~/pluto
git clone -b dashboard-integration https://github.com/techsavvyomi/pluto_ros2_package.git
cd pluto_ros2_package
git clone -b fix-1080p-framebuffer https://github.com/techsavvyomi/pluto_cam_ros2.git pluto_ros2_package/pluto_cam_ros2

# 3. Install camera deps (inside conda env)
conda activate ros2
mamba install -y -c conda-forge -c robostack-staging \
  ros-humble-cv-bridge ros-humble-image-transport opencv av

# 4. Build the workspace
./build.sh

# 5. Launch just the dashboard — the only terminal you need
./start_dashboard.sh
```

Open `http://localhost:5050` and use the **Services** panel to start plutonode / pluto_camera / pluto_vision — no extra terminals required.

---

## Architecture

```
┌──────────────────────────┐       ┌──────────────────────────┐
│ plutonode (plutodrone)   │       │ pluto_dashboard          │
│  • TCP MSP driver        │       │  • Flask @ :5050         │
│  • /drone_command  (sub) │◀──────│  • rclpy Node            │
│  • /pluto/imu      (pub) │──────▶│  • process manager       │
│  • /pluto/battery  (pub) │       │  • subprocess: drone/cam │
│  • /pluto/altitude (pub) │       └──────────────────────────┘
│  • /pluto/calibrate_*    │                   ▲ subscribes
│  • /pluto/motor_test     │       ┌──────────────────────────┐
│  • /pluto/set_pid        │       │ pluto_camera_sense       │
│  • auto-picks AP:        │       │  • /plutocamera/image_raw│
│    192.168.0.1:9060      │       │  • H.264 → BGR via ffmpeg│
│    192.168.4.1:23        │       └──────────────────────────┘
└──────────────────────────┘                   ▲
                                   ┌──────────────────────────┐
                                   │ pluto_vision (optional)  │
                                   │  • ArUco → /drone_command│
                                   └──────────────────────────┘
```

The dashboard is just one consumer — **any ROS node** can publish `/drone_command` and subscribe to the telemetry topics. See *Extending* below.

---

## Network modes

The Pluto has two radio configurations. The driver and dashboard auto-detect which one is live:

| Config | Wi-Fi to join | Camera | Flight controller |
|---|---|---|---|
| **No camera module** | `Pluto…` (drone's AP) | — | `192.168.4.1:23` (direct) |
| **Camera module attached** | `WIFI-1080p-…` | `192.168.0.1:8065` | `192.168.0.1:9060` (bridged through camera) |

When the camera is attached the drone's own AP is **off**; MSP commands flow through the camera module. `connectSock` tries camera-bridged first, then direct.

Override if needed: `PLUTO_HOST=192.168.x.y:port ./start_drone.sh`

---

## Topics published / consumed by `plutonode`

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `/drone_command` | `custom_msgs/PlutoMsg` | sub | Your RC out at ~20 Hz |
| `/pluto/imu` | `sensor_msgs/Imu` | pub | BEST_EFFORT, orientation quat + m/s² accel + rad/s gyro |
| `/pluto/battery` | `sensor_msgs/BatteryState` | pub | BEST_EFFORT, voltage + INA219 current + mAh + SoC |
| `/pluto/altitude` | `sensor_msgs/Range` | pub | BEST_EFFORT, barometer-derived (m) |
| `/pluto/calibrate_acc` | `std_msgs/Empty` | sub | Fires MSP_ACC_CALIBRATION |
| `/pluto/calibrate_mag` | `std_msgs/Empty` | sub | Fires MSP_MAG_CALIBRATION |
| `/pluto/eeprom_write` | `std_msgs/Empty` | sub | Persists current settings to EEPROM |
| `/pluto/motor_test` | `std_msgs/Int16MultiArray[4]` | sub | Spin M0..M3 — **props off!** |
| `/pluto/acc_trim` | `std_msgs/Int16MultiArray[2]` | sub | `[roll, pitch]` accel trim + EEPROM |
| `/pluto/set_pid` | `std_msgs/Int16MultiArray[9]` | sub | `[rP rI rD pP pI pD yP yI yD]` + EEPROM |

---

## Dashboard walkthrough

**Fly tab** — Services panel, arm/takeoff/land, input mode (None / Keyboard / Gamepad / Dev), sticks with sensitivity, camera preview, live telemetry + attitude graph.

**Calibrate tab** — accel / mag buttons (runs with wizard prompts), accel trim sliders → Apply.

**Gamepad setup** — live view of all detected axes + buttons. *Detect* auto-assigns a channel when you wiggle the stick; *Bind* captures a button press OR a switch flick (works for 3-pos switches on RC TXs like RadioMaster / FrSky).

**Keyboard setup** — per-channel `+ / –` key bindings; remappable action keys (arm, takeoff, land, calibrate, etc.).

**Logs tab** — start/stop CSV recording. One row per second with timestamp, full telemetry, RC channels, and current PID gains.

**Advanced tab** — PID table with Apply (writes `MSP_SET_PID` + `EEPROM_WRITE`); motor test sliders (requires props-off confirmation).

### Hot keys
- **Esc** triggers emergency stop (works in any input mode).
- Stick and action keys are configurable in the Keyboard setup tab.

---

## Foxglove layout

`foxglove_layout.json` at the repo root imports into [Foxglove Studio](https://foxglove.dev/) and gives you an instrument panel: 3D IMU view, camera, attitude / gyro / altitude / battery plots, SoC indicator. Open Foxglove → Layout → Import from file → pick this JSON. Zero code.

---

## Extending (using the stack without the dashboard)

The dashboard is optional. A custom ROS node just needs:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, BatteryState, Range
from custom_msgs.msg import PlutoMsg

class MyController(Node):
    def __init__(self):
        super().__init__('my_controller')
        self.rc_pub = self.create_publisher(PlutoMsg, '/drone_command', 10)
        self.create_subscription(Imu,          '/pluto/imu',      self.on_imu,  10)
        self.create_subscription(BatteryState, '/pluto/battery',  self.on_bat,  10)
        self.create_subscription(Range,        '/pluto/altitude', self.on_alt,  10)
        self.create_timer(0.05, self.tick)   # 20 Hz RC publish

    def tick(self):
        msg = PlutoMsg()
        msg.rc_roll = msg.rc_pitch = msg.rc_yaw = 1500
        msg.rc_throttle = 1500
        # AUX4=1500 → armed, 1200 → disarmed (Pluto scheme)
        msg.rc_aux4 = 1200
        self.rc_pub.publish(msg)
```

Record a full flight in one line:
```bash
ros2 bag record /drone_command /pluto/imu /pluto/battery /pluto/altitude /plutocamera/image_raw
```

---

## Pluto arming (AUX4-latched)

Pluto uses an AUX4-switch arming scheme (not the classic MultiWii yaw+throttle combo):

| Action | `throttle` | `aux4` |
|---|---|---|
| Disarm | 1300 | 1200 |
| Arm (low throttle) | 1000 | 1500 |
| Box arm (higher idle) | 1800 | 1500 |
| Takeoff | disarm → box_arm → `command_type=1` |
| Land | `command_type=2` |

Values are **latched** — the dashboard keeps publishing the last values at 20 Hz so the firmware sees them continuously. Resetting sticks to neutral mid-flight will not disarm by itself; only `aux4=1200` (+ `throttle=1300`) disarms.

---

## Flight log CSV columns

Each row is 1 Hz sample:

```
timestamp, battery, alt, roll, pitch, yaw,
acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z,
current_ma, mah_drawn, mah_remain, soc, auto_land,
rc_roll, rc_pitch, rc_yaw, rc_throttle,
rc_aux1, rc_aux2, rc_aux3, rc_aux4,
pid_roll_p, pid_roll_i, pid_roll_d,
pid_pitch_p, pid_pitch_i, pid_pitch_d,
pid_yaw_p, pid_yaw_i, pid_yaw_d
```

Units: V, m, °, g, °/s, raw mag, mA, mAh, %, 1000–2000. Saved under `logs/flight_YYYYMMDD_HHMMSS.csv`.

---

## Scripts

| Script | Purpose |
|---|---|
| `bootstrap_ros.sh` | First-time miniforge + RoboStack Humble install |
| `build.sh` | Colcon build with conda Python hints + ament-hook patches |
| `start_dashboard.sh` | **The only one you normally need** — dashboard launches the rest |
| `start_drone.sh` | Standalone flight driver (also invoked by Services panel) |
| `start_camera.sh` | Standalone camera publisher |
| `start_vision.sh` | Standalone ArUco follower (starts disabled) |

---

## Known limitations / roadmap

- **Upstream `pluto_cam_ros2`** has a hardcoded 1280×720 frame buffer that drops 1080p frames silently. Use our [`fix-1080p-framebuffer` branch](https://github.com/techsavvyomi/pluto_cam_ros2/tree/fix-1080p-framebuffer) which adds `--width/--height` args and an ffmpeg scale filter.
- **PID read-back** (`MSP_PID`) isn't parsed yet; dashboard shows the last values you applied, not what firmware currently has.
- **Autopilot** (`PlutoMsgAP`) subscriber exists in plutonode but isn't surfaced in the UI.

---

## License

MIT on our additions. Upstream `plutodrone`, `plutoserver`, `custom_msgs` retain their original DronaAviation licenses.
