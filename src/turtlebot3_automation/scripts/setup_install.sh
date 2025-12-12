#!/usr/bin/env bash
set -e

echo "========== [TB3 SETUP] Installing dependencies =========="

sudo apt update -y
sudo apt upgrade -y

sudo apt install -y \
  git curl wget \
  python3-pip python3-venv \
  python3-rosdep \
  python3-colcon-common-extensions

sudo rosdep init 2>/dev/null || true
rosdep update

sudo apt install -y \
  ros-humble-turtlebot3* \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-turtlebot3-simulations

python3 -m pip install --upgrade pip
python3 -m pip install opencv-python

echo "========== [DONE] System setup finished =========="
