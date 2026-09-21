# M5–M6 — Localization Preparation and Integration

## Goal

Prepare the drone platform for LiDAR-inertial localization and verify that the required sensor data, timing, coordinate frames, and ROS 2 interfaces were suitable for integration with Point-LIO.

## Main Components

- 3D LiDAR
- IMU
- ROS 2 Jazzy
- Point-LIO ROS 2
- `lidar_time_converter`
- Custom sensor and TF configuration

## Implementation

The localization pipeline required several integration steps before Point-LIO could be used reliably.

Work during these milestones included:

- Verifying LiDAR point-cloud output
- Verifying IMU output
- Confirming frame IDs and TF relationships
- Preparing Point-LIO launch and configuration files
- Matching the simulated LiDAR layout to the localization configuration
- Correcting LiDAR timestamp handling
- Testing ROS 2 simulation time compatibility
- Checking the LiDAR-to-IMU extrinsic relationship
- Preparing the data pipeline for real-time LiDAR-inertial estimation
A custom ROS 2 node named `lidar_time_converter` was added to handle timestamp compatibility between the simulated LiDAR output and Point-LIO.

## Validation

The localization input pipeline was considered ready when:

1. LiDAR data was published continuously.
2. IMU data was published continuously.
3. Point-LIO could subscribe to the required sensor topics.
4. Sensor timestamps were accepted correctly.
5. TF and sensor extrinsics were consistent.
6. Point-LIO could begin producing localization output without immediate integration errors.

## Result

**PASSED**

M5–M6 established the sensor, timing, and configuration foundation required for M7 GPS-denied localization and flight.
