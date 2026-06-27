#!/bin/bash
set -e

echo "=== Updating package list ==="
apt-get update

echo "=== Installing locales ==="
apt-get install -y locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "=== Installing software-properties-common ==="
apt-get install -y software-properties-common
add-apt-repository -y universe

echo "=== Adding ROS 2 GPG key ==="
apt-get update && apt-get install -y curl gnupg
mkdir -p /usr/share/keyrings
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "=== Adding ROS 2 repository to sources list ==="
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list

echo "=== Updating package list ==="
apt-get update

echo "=== Installing ROS 2 Humble and Build Dependencies ==="
# Install ROS2 humble, build tools, message generation packages, and camera packages
apt-get install -y \
  ros-humble-ros-base \
  ros-humble-sensor-msgs \
  ros-humble-std-msgs \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-rosidl-default-generators \
  ros-humble-rosidl-default-runtime \
  python3-colcon-common-extensions \
  python3-pip \
  python3-opencv \
  python3-av \
  python3-flask \
  build-essential

echo "=== Installing python requirements ==="
python3 -m pip install --upgrade pip
python3 -m pip install flask Werkzeug==2.2.2  # Werkzeug 2.2.2 is safe for older Flask/extensions if needed, or default is fine. Let's just install flask.

echo "=== Sourcing ROS 2 Humble ==="
if ! grep -q "source /opt/ros/humble/setup.bash" /root/.bashrc; then
  echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
fi

echo "=== ROS 2 Humble Installation Complete! ==="
