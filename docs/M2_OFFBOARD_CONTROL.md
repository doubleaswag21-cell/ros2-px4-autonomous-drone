# M2 — ROS 2 Offboard Control

## Goal

Enable autonomous flight control from ROS 2 by commanding PX4 in Offboard mode.

## Main Components

- ROS 2 Jazzy
- PX4 SITL
- `px4_msgs`
- `px4_ros_com`
- Micro XRCE-DDS Agent
- Custom `drone_offboard_control` ROS 2 package

## Implementation

The official PX4 ROS 2 Offboard example was first used to verify that Offboard control worked correctly.

After that validation, a custom ROS 2 node named `offboard_control` was developed inside the `drone_offboard_control` package.

The controller publishes the required PX4 Offboard control messages and vehicle trajectory setpoints through the `/fmu/in/*` interface.

The basic mission sequence included:

1. Publish Offboard control setpoints.
2. Request PX4 Offboard mode.
3. Arm the vehicle.
4. Take off.
5. Hold position.
6. Fly to a commanded position.
7. Return toward the starting position.
8. Land.
## Validation

M2 was validated in two stages.

### M2A — Official PX4 Example

The official PX4 ROS 2 Offboard example successfully controlled the simulated quadcopter.

### M2B — Custom Offboard Controller

The custom `offboard_control` ROS 2 node successfully executed an autonomous flight mission using PX4 Offboard mode.

The node can be launched with:

`ros2 run drone_offboard_control offboard_control`

## Result

**PASSED**

M2 established the custom ROS 2 flight-control foundation used by the later mapping, localization, and GPS-denied navigation milestones.
