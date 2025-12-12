# TurtleBot3 Automation Project (ROS2)

## Overview
This project implements a complete automation system for TurtleBot3 using ROS2 Humble. The system integrates setup automation, maintenance monitoring, autonomous navigation, object detection, and a custom safety feature. All components are designed as modular ROS2 nodes and can be launched either individually or through a single automation script.

---

## System Architecture

The automation system is composed of the following modules:

- **Setup Automation** – ROS2 workspace and environment configuration
- **Maintenance Automation** – Health and data availability monitoring
- **Navigation Automation** – Autonomous navigation using Nav2 and SLAM
- **Object Detection** – Real-time obstacle detection using LiDAR data
- **Custom Safety Feature** – Automatic robot stop based on detected obstacles

Each module runs as an independent ROS2 node and communicates using ROS2 topics and actions.

---

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble Hawksbill
- Gazebo (Classic)
- TurtleBot3 packages
- Python 3

Install required dependencies:

```
sudo apt update
sudo apt install ros-humble-turtlebot3* ros-humble-navigation2 ros-humble-nav2-bringup
```

---

## Workspace Setup

Create and build the ROS2 workspace:

```
mkdir -p ~/turtlebot3_automation_ws/src
cd ~/turtlebot3_automation_ws
colcon build
source install/setup.bash
```

---

## Package Structure

```
turtlebot3_automation
├── maintenance_node.py
├── navigation_manager.py
├── object_detection_node.py
├── safety_stop_node.py
├── __init__.py
setup.py
package.xml
```

---

## Automation Script

The entire system can be launched using a single automation script, structured similarly to the smart lighting automation file used earlier in the course.

```
./turtlebot3_automation_manager.sh
```

This script automatically launches:
- Gazebo simulation with TurtleBot3
- Navigation stack (Nav2 + SLAM)
- Maintenance monitoring node
- Object detection node
- Safety stop node

---

## Running the System Manually

Each component can also be launched in a separate terminal if manual control is preferred.

---

### 1. Launch TurtleBot3 Simulation (Gazebo)

```
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

---

### 2. Launch Navigation (Nav2 + SLAM)

```
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py slam:=True
```

---

### 3. Maintenance Automation Node

```
source /opt/ros/humble/setup.bash
source ~/turtlebot3_automation_ws/install/setup.bash
ros2 run turtlebot3_automation maintenance_node
```

**Description:**  
Monitors robot data availability and logs warnings when expected data is missing.

---

### 4. Object Detection Node (LiDAR-based)

```
source /opt/ros/humble/setup.bash
source ~/turtlebot3_automation_ws/install/setup.bash
ros2 run turtlebot3_automation object_detection_node
```

**Description:**  
Subscribes to `/scan`, detects obstacle clusters within a defined distance, and publishes detection results to `/detections`.

To view detections:

```
ros2 topic echo /detections
```

---

### 5. Custom Feature – Safety Stop Node

```
source /opt/ros/humble/setup.bash
source ~/turtlebot3_automation_ws/install/setup.bash
ros2 run turtlebot3_automation safety_stop_node
```

**Description:**  
Listens to `/detections`. When obstacles are detected, the robot is stopped by publishing zero velocity to `/cmd_vel`, and an alert is published to `/health_alerts`.

---

### 6. Navigation Automation Node

```
source /opt/ros/humble/setup.bash
source ~/turtlebot3_automation_ws/install/setup.bash
ros2 run turtlebot3_automation navigation_manager \
  --ros-args -p goal_x:=1.0 -p goal_y:=0.0 -p goal_yaw_deg:=0.0
```

**Description:**  
Sends navigation goals programmatically, removing the need for manual goal selection in RViz.

---

## RViz Notes

RViz may display warnings related to missing optional Nav2 UI plugins. These warnings do not affect navigation, mapping, or robot control functionality and can be safely ignored.

---

## Assignment Requirement Mapping

| Requirement | Status |
|------------|--------|
| Setup Automation | Completed |
| Maintenance Automation | Completed |
| Navigation Automation | Completed |
| Object Detection Integration | Completed |
| Custom Feature | Completed |

---

## Conclusion

This project demonstrates a complete ROS2-based TurtleBot3 automation system with integrated navigation, perception, and safety mechanisms. The system was developed using official ROS2 and TurtleBot3 tools and verified through simulation.

