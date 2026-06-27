# pluto_camera_sense

ROS 2 package that connects to the **PlutoCam** drone camera over WiFi and publishes live video on the ROS 2 topic `/plutocamera/image_raw` as `sensor_msgs/Image`.

## Requirements

| Dependency | Install |
|---|---|
| Python `rclpy` | Comes with ROS 2 Humble |
| `cv_bridge` | `sudo apt install ros-humble-cv-bridge` |
| `sensor_msgs` | Comes with ROS 2 Humble |
| `ffmpeg` | `sudo apt install ffmpeg` (must be on PATH) |
| **WiFi connection** | Connect laptop to the drone camera's WiFi AP (`WIFI-1080p-…`). Camera IP is `192.168.0.1`. |

## Usage

### Start camera node alone
```bash
ros2 launch pluto_camera_sense camera_launch.py
# Use a different IP:
ros2 launch pluto_camera_sense camera_launch.py cam_ip:=192.168.0.1
```

### Start camera + dashboard together
```bash
ros2 launch pluto_dashboard dashboard_launch.py
```
The dashboard will automatically display the camera feed in the video box.

## Topic published

| Topic | Type | Description |
|---|---|---|
| `/plutocamera/image_raw` | `sensor_msgs/Image` | Live H.264-decoded video frames (bgr8 encoding) |

## Parameters / arguments

| Argument | Default | Description |
|---|---|---|
| `--ip` | `192.168.0.1` | Camera IP address |
| `--low-def` | off | Use 720p instead of 1080p |
| `--display` | off | Open ffplay window for local display |
