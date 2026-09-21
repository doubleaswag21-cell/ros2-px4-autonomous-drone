# M0 — PX4 SITL + Gazebo Flight

## Goal

Establish a working PX4 Software-In-The-Loop simulation environment and verify that a quadcopter could arm, take off, hover, and land in Gazebo.

## Main Components

- PX4 Autopilot SITL
- Gazebo Harmonic
- QGroundControl
- X500 quadcopter model

## Implementation

PX4 SITL was built and launched with Gazebo Harmonic.

QGroundControl was used to connect to PX4 through UDP and verify vehicle status, arming, takeoff, hover, and landing behavior.

## Validation

The simulated quadcopter successfully:

1. Connected to QGroundControl
2. Passed PX4 preflight checks
3. Armed
4. Took off
5. Hovered
6. Landed safely

## Result

**PASSED**

M0 established the base flight simulation environment used for all later ROS 2, SLAM, and autonomous navigation work.
