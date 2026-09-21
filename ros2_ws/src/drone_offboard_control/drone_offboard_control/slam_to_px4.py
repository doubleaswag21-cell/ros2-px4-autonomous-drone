#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry


class SlamToPx4(Node):

    def __init__(self):
        super().__init__('slam_to_px4')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub = self.create_publisher(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            px4_qos
        )

        self.sub = self.create_subscription(
            Odometry,
            '/aft_mapped_to_init',
            self.odom_callback,
            10
        )

        self.count = 0

        self.get_logger().info(
            'M7F Point-LIO -> PX4 bridge started'
        )
        self.get_logger().info(
            'Input: /aft_mapped_to_init'
        )
        self.get_logger().info(
            'Output: /fmu/in/vehicle_visual_odometry'
        )
        self.get_logger().info(
            'ROS ENU -> PX4 NED'
        )

    def safe_variance(self, value, fallback=0.04):

        if math.isfinite(value) and value > 0.0:
            return max(float(value), 0.01)

        return fallback

    def odom_callback(self, odom):

        msg = VehicleOdometry()

        # -------------------------------------------------
        # Timestamp
        # -------------------------------------------------

        sample_ns = (
            odom.header.stamp.sec * 1_000_000_000
            + odom.header.stamp.nanosec
        )

        sample_us = int(sample_ns / 1000)

        now_us = int(
            self.get_clock().now().nanoseconds / 1000
        )

        # Protect PX4 from simulation-clock startup/reset
        # anomalies or unexpectedly stale estimator data.
        if now_us <= 0:
            return

        if sample_us > now_us:
            self.get_logger().warn(
                'Point-LIO timestamp is ahead of ROS clock - '
                'dropping sample'
            )
            return

        age_us = now_us - sample_us

        if age_us > 200000:
            self.get_logger().warn(
                f'Point-LIO odometry is stale '
                f'({age_us / 1000.0:.1f} ms) - dropping sample'
            )
            return

        msg.timestamp = now_us
        msg.timestamp_sample = sample_us

        # -------------------------------------------------
        # Position:
        # ROS local FLU -> PX4 local FRD
        #
        # X forward stays X
        # Y left    -> Y right
        # Z up      -> Z down
        # -------------------------------------------------

        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        z = odom.pose.pose.position.z

        msg.pose_frame = VehicleOdometry.POSE_FRAME_NED

        # ROS / Point-LIO ENU -> PX4 NED
        # ROS X (East)  -> PX4 Y (East)
        # ROS Y (North) -> PX4 X (North)
        # ROS Z (Up)    -> PX4 Z (Down)

        msg.position = [
            float(y),
            float(x),
            float(-z)
        ]

        # -------------------------------------------------
        # Position covariance
        # -------------------------------------------------

        cov = odom.pose.covariance

        msg.position_variance = [
            self.safe_variance(cov[7]),   # ROS Y -> PX4 X
            self.safe_variance(cov[0]),   # ROS X -> PX4 Y
            self.safe_variance(cov[14])   # ROS Z -> PX4 Z
        ]

        # -------------------------------------------------
        # POSITION ONLY FOR FIRST PX4 EKF TEST
        #
        # We are intentionally NOT fusing external yaw
        # or velocity yet.
        # -------------------------------------------------

        nan = float('nan')

        msg.q = [
            nan,
            nan,
            nan,
            nan
        ]

        msg.velocity_frame = (
            VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        )

        msg.velocity = [
            nan,
            nan,
            nan
        ]

        msg.angular_velocity = [
            nan,
            nan,
            nan
        ]

        msg.orientation_variance = [
            nan,
            nan,
            nan
        ]

        msg.velocity_variance = [
            nan,
            nan,
            nan
        ]

        msg.reset_counter = 0
        msg.quality = 100

        self.pub.publish(msg)

        self.count += 1

        if self.count % 30 == 0:

            age_ms = (
                msg.timestamp - msg.timestamp_sample
            ) / 1000.0

            self.get_logger().info(
                f'PX4 NED '
                f'[{msg.position[0]:.3f}, '
                f'{msg.position[1]:.3f}, '
                f'{msg.position[2]:.3f}] | '
                f'age={age_ms:.1f} ms | '
                f'var=['
                f'{msg.position_variance[0]:.4f}, '
                f'{msg.position_variance[1]:.4f}, '
                f'{msg.position_variance[2]:.4f}]'
            )


def main(args=None):

    rclpy.init(args=args)

    node = SlamToPx4()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
