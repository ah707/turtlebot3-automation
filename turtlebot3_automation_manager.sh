#!/bin/bash

# ================================
# TurtleBot3 Automation Manager
# ROS2 Humble
# ================================

echo "Starting TurtleBot3 Automation System..."

# ---- Safety check ----
if [ -z "$ROS_DISTRO" ]; then
  echo "ROS2 environment not sourced. Exiting."
  exit 1
fi

# ---- Robot model ----
export TURTLEBOT3_MODEL=burger

# ---- Source environments ----
source /opt/ros/humble/setup.bash
source ~/turtlebot3_automation_ws/install/setup.bash

# ---- Launch Gazebo ----
echo "Launching Gazebo simulation..."
gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash;
export TURTLEBOT3_MODEL=burger;
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py;
exec bash" &

sleep 10

# ---- Launch Navigation (Nav2 + SLAM) ----
echo "Launching Navigation stack..."
gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash;
export TURTLEBOT3_MODEL=burger;
ros2 launch turtlebot3_navigation2 navigation2.launch.py slam:=True;
exec bash" &

sleep 10

# ---- Maintenance Node ----
echo "Starting maintenance node..."
gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash;
source ~/turtlebot3_automation_ws/install/setup.bash;
ros2 run turtlebot3_automation maintenance_node;
exec bash" &

sleep 2

# ---- Object Detection Node ----
echo "Starting object detection node..."
gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash;
source ~/turtlebot3_automation_ws/install/setup.bash;
ros2 run turtlebot3_automation object_detection_node;
exec bash" &

sleep 2

# ---- Safety Stop Node ----
echo "Starting safety stop node..."
gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash;
source ~/turtlebot3_automation_ws/install/setup.bash;
ros2 run turtlebot3_automation safety_stop_node;
exec bash" &

echo "All automation components launched successfully."
echo "System is now running."
