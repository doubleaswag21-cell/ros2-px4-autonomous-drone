# M3 — Custom `slam_quad` Vehicle

## Goal

Develop a custom PX4/Gazebo quadcopter model suitable for later SLAM, sensing, and autonomous navigation work.

## Main Components

- PX4 SITL
- Gazebo Harmonic
- Custom `slam_quad` model
- Custom `slam_quad_base` model
- PX4 custom airframe configuration

## Implementation

A dedicated Gazebo vehicle named `slam_quad` was created instead of continuing with the stock X500 model.

The custom model included:

- Custom body geometry
- Extended arms
- Landing skids
- Updated collision geometry
- Updated mass and inertia
- Wider rotor geometry
- Matching PX4 control-allocation parameters

A dedicated PX4 airframe configuration was also created:
`22000_gz_slam_quad`

The airframe defines the custom rotor positions, motor directions, actuator mappings, and hover-thrust settings required by the new geometry.

The custom simulation target was launched using:

`make px4_sitl gz_slam_quad`

## Validation

The custom vehicle was tested independently before adding the full autonomy stack.

Validation included:

1. PX4 startup with the custom airframe.
2. Successful Gazebo spawning.
3. Successful preflight checks.
4. Manual arming.
5. Stable takeoff.
6. Stable hover.
7. Controlled landing.
8. No obvious instability caused by the modified geometry.

## Result

**PASSED**

M3 established the physical simulation platform used by the later sensor, SLAM, and GPS-denied autonomy milestones.
