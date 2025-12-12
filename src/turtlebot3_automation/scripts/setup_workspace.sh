#!/usr/bin/env bash
set -e

echo "========== [TB3 SETUP] Building workspace =========="

WS=~/turtlebot3_automation_ws

source /opt/ros/humble/setup.bash
cd "$WS"

rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build

echo "========== [DONE] Workspace build finished =========="
echo "Run: source ~/turtlebot3_automation_ws/install/setup.bash"
