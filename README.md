# f450_drone_demo

## Demo

![MAVLink → ROS 2 Telemetry Pipeline](media/mavlink_ros2_demo.gif)

*ESP32 streams MAVLink attitude data via UDP to MAVROS, displayed in RViz2 with HUD and F450 model*

## Prerequisites

- ROS 2 Humble
- MAVROS package (`apt install ros-humble-mavros-*`)
- ESP32 with WiFi capability
- Wireshark (optional, for packet inspection)

## Overview

Hardware-in-the-loop bench test for validating MAVLink → ROS 2 communication pipeline before any real flight. An ESP32-WROOM emulates MAVLink attitude data and streams it over UDP to MAVROS, which converts NED → ENU frames and publishes standard ROS 2 topics for visualization in RViz2.

**Pipeline:** ESP32 (MAVLink) → UDP → MAVROS (ROS 2) → /mavros/imu/data → RViz2 (HUD + F450 model)

## Why This Approach

This approach validates communication, message rates, reference frames, and visualization with zero propeller spin. Validating telemetry pipelines in hardware-in-the-loop configuration reduces risk and accelerates development.

## Technologies Demonstrated

- **ROS 2** (Humble) - launch files, custom nodes, topic remapping
- **MAVLink protocol** - attitude message generation and parsing
- **MAVROS** - NED ↔ ENU frame transformations
- **ESP32** - UDP networking, embedded systems integration
- **URDF/xacro** - robot description and visualization
- **RViz2** - custom plugins, 3D visualization

## Technical Challenges

- **Frame convention validation**: Verified NED→ENU conversion with systematic roll/pitch/yaw testing
- **QoS profile matching**: Ensured MAVROS topics compatible with custom ROS2 nodes
- **Real-time visualization**: Integrated third-party RViz plugin with custom telemetry pipeline

## Workspace Structure

- `src/esp32_mavlink_bridge`: emulates and relays MAVLink UDP packets to MAVROS
- `src/f450_drone_bridge`: MAVROS-based attitude visualization (RViz2)
- `src/f450_drone_description`: F450 quadcopter URDF (xacro) description 
- `src/rviz_attitude_plugin`: HUD component plugin for RViz (external git repo by Abdelrahman Mahmoud)

## Launch Commands

**1. Start MAVROS bridge + telemetry processing:**

Launches the MAVROS bridge (UDP port 14550), MarkerArray generation for cardinal points, TF generation for attitude display, and IMU topic roll corrections for the attitude plugin:
```bash
ros2 launch f450_drone_bridge mavros_and_path.launch.py fcu_url:=udp://@0.0.0.0:14550?mav10=1
```

**2. Start robot description + RViz2 visualization:**

Launches xacro processing, robot state publisher (RSP), and RViz2:
```bash
ros2 launch f450_drone_description display.launch.py
```

**3. Monitor MAVLink UDP packets (optional):**

Display live MAVLink traffic with Wireshark:
```bash
sudo wireshark -i wlp3s0 -k -f "udp port 14550"
```

## What the Demo Shows

- **Live attitude tracking** — HUD and F450 model update in real time as ESP32 streams MAVLink data
- **Frame conversion** — MAVROS converts NED (North–East–Down) to ENU (East–North–Up) for ROS 2
- **Consistent orientation** — pitch, roll, and yaw appear exactly as expected in the ENU frame
- **Clear visualization** — attitude plugin provides an intuitive view of the drone's state


## Contact

LinkedIn: https://www.linkedin.com/in/alain-meil-50b9525

*Currently seeking robotics engineering opportunities in autonomous navigation, UAV systems, or sensor integration.*