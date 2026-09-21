# M1 — ROS 2 ↔ PX4 Communication

## Goal

Establish reliable communication between ROS 2 and PX4 so that ROS 2 nodes could receive PX4 vehicle data and later send control commands.

## Main Components

- ROS 2 Jazzy
- PX4 SITL
- `px4_msgs`
- `px4_ros_com`
- Micro XRCE-DDS Agent

## Implementation

The ROS 2 workspace was kept separate from the PX4 source tree and configured with the required PX4 message packages.

The Micro XRCE-DDS Agent was launched to bridge PX4 uORB data into ROS 2 DDS topics.

After sourcing the ROS 2 workspace, PX4 topics became visible under `/fmu/in/*` and `/fmu/out/*`.

Important vehicle data such as `/fmu/out/vehicle_odometry` could then be accessed from ROS 2.

## Validation

Communication was considered successful when:

1. PX4 SITL was running.
2. The Micro XRCE-DDS Agent connected successfully.
3. ROS 2 discovered the PX4 topics.
4. `/fmu/out/vehicle_odometry` was visible and publishing.
5. ROS 2 could access PX4 state information without communication errors.

## Result

**PASSED**

M1 established the ROS 2 ↔ PX4 communication layer required for later Offboard control and autonomous flight.
