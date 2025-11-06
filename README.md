# f450_drone_ws

ROS 2 workspace for F450 drone demos:
- `src/f450_bridges`: MAVROS-based trajectory visualization (RViz2)
- More packages to come

## run telemetry
ros2 launch f450_bridges mavros_and_path.launch.py frame:=map

## run (serial alternative)
ros2 launch f450_bridges mavros_and_path.launch.py frame:=map fcu_url:=serial:///dev/ttyUSB0:57600
