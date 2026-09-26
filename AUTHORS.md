# Authors and Engineering Contributions

## Project Developer

**Ammaar Ahmed**

ROS 2 + PX4 Autonomous GPS-Denied Drone

GitHub: doubleaswag21-cell

## Original Engineering Contributions

### Custom C++ 3D A* Planner

Developed the project-specific ROS 2 3D path-planning package, including:

- `astar_3d`: three-dimensional A* path planning using OctoMap.
- `octomap_query`: interactive occupancy-map querying.
- Horizontal and vertical obstacle-clearance constraints.
- Final path validation and ROS 2 path publication.
Location: `ros2_ws/src/drone_3d_planner/`

### Custom ROS 2 Sensor Description

Developed the project's URDF/Xacro sensor-frame description,
including the drone body, LiDAR, RGB-D camera, and optical frames.

Location: `ros2_ws/src/slam_quad_description/`

### Gazebo Simulation Environments

Designed and configured project-specific simulation environments,
including the larger geometrically rich environment developed
for M10 localization testing.

Location: `gazebo/worlds/`

### Custom Quadcopter Integration

Customized and integrated the `slam_quad` vehicle for PX4 SITL
and Gazebo simulation, including project-specific geometry,
physical configuration, sensors, and vehicle integration.

The vehicle incorporates existing third-party model assets.
Authorship of those original assets is not claimed.
### ROS 2 and PX4 System Integration

Developed and integrated project-specific functionality for
autonomous flight, GPS-denied localization, occupancy mapping,
mission execution, and experimental validation.

## Third-Party Acknowledgments

This project uses and integrates external open-source software,
including PX4 Autopilot, ROS 2, Gazebo, Point-LIO, and OctoMap.

The original quadcopter model assets include material copyrighted
by Rudis Laboratories and distributed under the BSD 3-Clause
license.

Existing third-party copyrights and licenses remain applicable.

This document identifies project-specific contributions and
does not claim ownership of third-party software, algorithms,
or original model assets.

## Licensing

This repository contains original project contributions
alongside third-party open-source components.

Existing third-party licenses remain applicable, including
the BSD 3-Clause license for the original quadcopter model
assets.

The slam_quad_description package declares Apache 2.0.

No repository-wide license has currently been selected.
The original drone_3d_planner implementation and custom
simulation environments have not yet been assigned
individual open-source licenses.

Public availability does not automatically grant permission
to redistribute or commercially reuse unlicensed original
material.
