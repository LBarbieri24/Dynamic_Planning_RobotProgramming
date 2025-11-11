# RP_project / dynamic_planner
https://github.com/user-attachments/assets/7aa7e858-59f5-4a7a-ab44-02bf8ae1bd17

This repository contains a small ROS 2 package `dynamic_planner` that implements a grid-based A* dynamic planner with an occupancy/distance map computed from LaserScan data. It was developed as part of a robotics project.


## Repository layout

- `src/dynamic_planner/` — ROS 2 package implementing the planner and node:
  - `src/` — C++ sources
  - `dynamic_planner_node.cpp` — ROS node: subscribes to `/scan`, maintains local/global maps, runs the planner periodically and publishes `/planner/path` and `/planner/distance_map`.
  - `planner.cpp` — A* implementation, heuristic, cost conversion.
  - `map.cpp` — grid map, LaserScan to local distance map conversion, fuse & distance transform.
  - `fake_laser_scan.cpp` — test node that publishes synthetic `sensor_msgs/LaserScan` messages for local testing and simulation.
  - `fake_robot_motion.cpp` — simple motion simulator/publisher that moves a fake robot pose (used together with the fake scan publisher for offline testing and visualization).
  - `include/dynamic_planner/` — public headers (`planner.hpp`, `map.hpp`, ...)
  - `CMakeLists.txt`, `package.xml`

- `RP_RVIZ.rviz` — RViz configuration used for visualization.

## Dependencies

This package is a ROS 2 (ament_cmake) package and depends on the following (declared in `package.xml`):

- build tool: `ament_cmake`
- runtime: `rclcpp`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `tf2`, `tf2_ros`, `std_msgs`


## Build instructions

From the repository root (where this README lives):

```bash
# (optional) source your ROS2 installation
source /opt/ros/<distro>/setup.bash

# build with colcon
colcon build --symlink-install

# source the overlay (after successful build)
source install/setup.bash
```

Note: the project already includes `install/local_setup.*` scripts in the `install/` folder after a build; you can source `install/setup.bash` for a one-off session.

## Run

A launch file is available (`launch/dynamic_planner.launch.py`) — run with:

```bash
source install/setup.bash    
ros2 launch dynamic_planner dynamic_planner.launch.py
```

You can visualize the path and maps with RViz. The repo includes `RP_RVIZ.rviz` which you can load into RViz.
