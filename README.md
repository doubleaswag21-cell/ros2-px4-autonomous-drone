# ROS 2 + PX4 Autonomous GPS-Denied Drone

A ROS 2 Jazzy and PX4-based autonomous quadcopter platform developed for GPS-denied localization, mapping, navigation, and indoor exploration.

The project uses a custom quadcopter model in Gazebo Harmonic, PX4 SITL for flight control, ROS 2 for autonomy and middleware, and Point-LIO for LiDAR-inertial localization.

## Current Status

Milestones M0-M8 are complete.

The current system supports:

- PX4 SITL flight in Gazebo Harmonic
- ROS 2 ↔ PX4 communication through Micro XRCE-DDS
- Custom ROS 2 Offboard flight control
- Custom `slam_quad` Gazebo/PX4 vehicle model
- 3D LiDAR, RGB-D camera, and IMU integration
- ROS 2 TF sensor-frame integration
- Point-LIO LiDAR-inertial localization
- Point-LIO odometry conversion for PX4 external vision
- PX4 external-vision fusion
- GPS-denied position hold
- GPS-denied autonomous waypoint navigation
- Autonomous landing and disarming
- OctoMap-based 3D occupancy mapping
- Persistent `.bt` map saving and reloading

## Latest Demonstration

![M8 OctoMap 3D occupancy mapping](images/octomap/m8_free_space.png)

Milestone 8 demonstrated live 3D occupancy mapping using Point-LIO and OctoMap.
The validated mapping pipeline is:

    Point-LIO
    /cloud_registered_body
            +
    camera_init → body TF
            ↓
    OctoMap Server
            ↓
    3D Occupancy Map

The map was generated at `0.10 m` voxel resolution and preserved open interior space while reconstructing room boundaries and internal obstacles.

The resulting OctoMap was also saved and successfully reloaded as:

    octomap/maps/slam_room_office_m8.bt

Full M8 documentation:

**[M8 — OctoMap 3D Occupancy Mapping](docs/M8_OCTOMAP_3D_MAPPING.md)**

## System Architecture

```text
Gazebo Harmonic
      |
      | 3D LiDAR + IMU + Camera
      v
    ROS 2
      |
      v
  Point-LIO
      |
      +-----------------------------+
      |                             |
      v                             v
LiDAR-Inertial                /cloud_registered_body
  Odometry                           |
      |                              v
      v                        OctoMap Server
 slam_to_px4                         |
      |                              v
      v                      3D Occupancy Map
PX4 External Vision
      |
      v
   PX4 EKF2
      |
      v
Offboard Controller
      |
      v
  slam_quad
```

## Coordinate Conversion

Point-LIO publishes odometry using the ROS ENU coordinate convention, while PX4 uses NED.

The position conversion used by the project is:

```text
PX4 X = ROS Y
PX4 Y = ROS X
PX4 Z = -ROS Z
```

## Software Stack

- Ubuntu Linux
- ROS 2 Jazzy
- PX4 Autopilot SITL
- Gazebo Harmonic
- Micro XRCE-DDS Agent
- Point-LIO
- OctoMap / octomap_server
- Python
- C++
- QGroundControl

## Repository Structure

```text
ros2-px4-autonomous-drone/
├── ros2_ws/
│   └── src/
│       ├── drone_offboard_control/
│       └── slam_quad_description/
│
├── px4/
│   └── slam_quad/
│       ├── airframe/
│       └── models/
│
├── point_lio/
│   ├── config/
│   └── launch/
│
├── gazebo/
│   └── worlds/
│
├── octomap/
│   └── maps/
│       └── slam_room_office_m8.bt
│
├── docs/
├── images/
└── scripts/
```

## Custom ROS 2 Nodes

The `drone_offboard_control` package currently contains:

- `offboard_control` — basic autonomous PX4 Offboard mission
- `mapping_mission` — autonomous flight mission for mapping tests
- `slam_to_px4` — converts SLAM odometry for PX4 external-vision fusion
- `lidar_time_converter` — converts LiDAR timestamps for Point-LIO compatibility
- `gps_denied_waypoint` — GPS-denied autonomous waypoint controller


## Project Milestones

- **M0** — PX4 SITL + Gazebo flight
- **M1** — ROS 2 ↔ PX4 communication
- **M2** — Autonomous Offboard control
- **M3** — Custom `slam_quad` vehicle
- **M4** — Sensor and TF integration
- **M5-M6** — Localization preparation and integration
- **M7** — GPS-denied localization and autonomous navigation
- **M8** — 3D occupancy mapping ✅ ([documentation](docs/M8_OCTOMAP_3D_MAPPING.md))
- **M9** — 3D path planning
- **M10** — Obstacle avoidance
- **M11** — Autonomous waypoint navigation
- **M12** — Frontier exploration
- **M13** — Full autonomous exploration
- **M14** — Optional semantic mapping

## Third-Party Components

This repository contains project-specific configuration and integration work for several open-source projects.

External dependencies include:

- PX4 Autopilot
- ROS 2
- Gazebo
- Micro XRCE-DDS
- Point-LIO
- OctoMap

Original third-party model assets retain their respective licenses and attribution. The `slam_quad` model directory includes the original BSD 3-Clause license associated with the upstream model assets.

## Development Status

This repository currently documents the project through **M8: OctoMap 3D occupancy mapping**.

Milestones M0-M8 are complete. Development continues with M9 and later milestones.

