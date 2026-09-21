#!/usr/bin/env python3

import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import PointCloud2, PointField


class LidarTimeConverter(Node):

    def __init__(self):
        super().__init__('lidar_time_converter')

        self.declare_parameter(
            'input_topic',
            '/slam_quad/lidar/points'
        )

        self.declare_parameter(
            'output_topic',
            '/slam_quad/lidar/points_timed'
        )

        self.declare_parameter(
            'scan_rate',
            10.0
        )

        self.input_topic = (
            self.get_parameter('input_topic')
            .get_parameter_value()
            .string_value
        )

        self.output_topic = (
            self.get_parameter('output_topic')
            .get_parameter_value()
            .string_value
        )

        self.scan_rate = (
            self.get_parameter('scan_rate')
            .get_parameter_value()
            .double_value
        )

        self.scan_period = 1.0 / self.scan_rate

        self.publisher = self.create_publisher(
            PointCloud2,
            self.output_topic,
            qos_profile_sensor_data
        )

        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_callback,
            qos_profile_sensor_data
        )

        self.first_cloud = True

        self.get_logger().info(
            'LiDAR time-field converter started'
        )

        self.get_logger().info(
            f'Input:  {self.input_topic}'
        )

        self.get_logger().info(
            f'Output: {self.output_topic}'
        )

        self.get_logger().info(
            f'Scan rate: {self.scan_rate:.1f} Hz'
        )

        self.get_logger().info(
            'Generated time field unit: seconds'
        )

    def cloud_callback(self, cloud):

        # -----------------------------------------------------
        # Avoid adding another time field if one already exists
        # -----------------------------------------------------

        existing_names = [field.name for field in cloud.fields]

        if 'time' in existing_names:
            self.get_logger().warn(
                'Input cloud already contains a time field. '
                'Passing cloud through unchanged.',
                throttle_duration_sec=5.0
            )

            self.publisher.publish(cloud)
            return

        width = cloud.width
        height = cloud.height

        if width == 0 or height == 0:
            return

        old_point_step = cloud.point_step

        # Add one FLOAT32 at the end of every point
        new_point_step = old_point_step + 4
        new_row_step = new_point_step * width

        # -----------------------------------------------------
        # Output PointCloud2 fields
        # -----------------------------------------------------

        new_fields = []

        for field in cloud.fields:
            copied_field = PointField()

            copied_field.name = field.name
            copied_field.offset = field.offset
            copied_field.datatype = field.datatype
            copied_field.count = field.count

            new_fields.append(copied_field)

        time_field = PointField()

        time_field.name = 'time'
        time_field.offset = old_point_step
        time_field.datatype = PointField.FLOAT32
        time_field.count = 1

        new_fields.append(time_field)

        # -----------------------------------------------------
        # Allocate output buffer
        # -----------------------------------------------------

        total_points = width * height

        new_data = bytearray(
            total_points * new_point_step
        )

        src = memoryview(cloud.data)

        float_format = (
            '>f' if cloud.is_bigendian else '<f'
        )

        # One scan = 0.1 s for our 10 Hz LiDAR.
        #
        # The organized cloud is:
        #
        #   height = 16   laser rings
        #   width  = 640  horizontal azimuth samples
        #
        # Points at the same horizontal column get the same
        # relative acquisition time.
        column_dt = self.scan_period / float(width)

        # -----------------------------------------------------
        # Copy original point data and append relative time
        # -----------------------------------------------------

        for row in range(height):

            for col in range(width):

                src_offset = (
                    row * cloud.row_step
                    + col * old_point_step
                )

                dst_index = row * width + col

                dst_offset = (
                    dst_index * new_point_step
                )

                # Copy original x/y/z/intensity/ring/etc.
                new_data[
                    dst_offset:
                    dst_offset + old_point_step
                ] = src[
                    src_offset:
                    src_offset + old_point_step
                ]

                # Relative acquisition time from beginning of the scan.
                # Gazebo simulation test: compress the full scan into 4 ms.
                scan_duration = 0.004
                column_dt = scan_duration / width
                relative_time = col * column_dt

                struct.pack_into(
                    float_format,
                    new_data,
                    dst_offset + old_point_step,
                    float(relative_time)
                )

        # -----------------------------------------------------
        # Build output cloud
        # -----------------------------------------------------

        output = PointCloud2()

        output.header = cloud.header

        output.height = height
        output.width = width

        output.fields = new_fields

        output.is_bigendian = cloud.is_bigendian

        output.point_step = new_point_step
        output.row_step = new_row_step

        output.data = bytes(new_data)

        output.is_dense = cloud.is_dense

        self.publisher.publish(output)

        if self.first_cloud:

            self.first_cloud = False

            last_time = 0.0

            self.get_logger().info(
                'First cloud converted successfully'
            )

            self.get_logger().info(
                f'Cloud size: {height} x {width}'
            )

            self.get_logger().info(
                f'Old point_step: {old_point_step} bytes'
            )

            self.get_logger().info(
                f'New point_step: {new_point_step} bytes'
            )

            self.get_logger().info(
                f'Column dt: {column_dt:.9f} s'
            )

            self.get_logger().info(
                f'Last point time: {last_time:.9f} s'
            )


def main(args=None):

    rclpy.init(args=args)

    node = LidarTimeConverter()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
