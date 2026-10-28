#!/usr/bin/env python3
# Copyright (c) 2026 Ammaar Ahmed
# Project-specific ROS 2 autonomy implementation.

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool


class LocalPathMonitor(Node):
    """Fast LiDAR corridor check for the exact path M13G is currently executing."""

    def __init__(self):
        super().__init__('local_path_monitor')

        self.cloud_topic = str(self.declare_parameter(
            'cloud_topic', '/cloud_registered').value)
        self.odom_topic = str(self.declare_parameter(
            'odom_topic', '/aft_mapped_to_init').value)
        self.path_topic = str(self.declare_parameter(
            'path_topic', '/m13g/active_path').value)
        self.blocked_topic = str(self.declare_parameter(
            'blocked_topic', '/local_path_blocked').value)
        self.frame_id = str(self.declare_parameter('frame_id', 'camera_init').value)

        self.lookahead_m = float(self.declare_parameter('lookahead_m', 2.0).value)
        self.horizontal_clearance = float(self.declare_parameter(
            'horizontal_clearance', 0.45).value)
        self.vertical_clearance = float(self.declare_parameter(
            'vertical_clearance', 0.25).value)
        self.sample_step_m = float(self.declare_parameter('sample_step_m', 0.10).value)
        self.local_radius_m = float(self.declare_parameter('local_radius_m', 3.0).value)
        self.min_points = int(self.declare_parameter('min_points', 3).value)
        self.point_stride = max(1, int(self.declare_parameter('point_stride', 2).value))
        self.block_confirm_scans = max(1, int(self.declare_parameter(
            'block_confirm_scans', 2).value))
        self.clear_confirm_scans = max(1, int(self.declare_parameter(
            'clear_confirm_scans', 3).value))

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5)
        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        self.block_pub = self.create_publisher(Bool, self.blocked_topic, reliable)
        self.create_subscription(PointCloud2, self.cloud_topic, self.cloud_cb, sensor_qos)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, sensor_qos)
        self.create_subscription(Path, self.path_topic, self.path_cb, latched)

        self.position = None
        self.path = None
        self.path_frame = None
        self.blocked = False
        self.block_count = 0
        self.clear_count = 0
        self.last_frame_warning = -1e9

        self.get_logger().info('Local 3D LiDAR path monitor ready')
        self.get_logger().info(
            f'Checking next {self.lookahead_m:.1f} m of {self.path_topic} at '
            f'XY={self.horizontal_clearance:.2f} m, Z={self.vertical_clearance:.2f} m')

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        xyz = np.array([float(p.x), float(p.y), float(p.z)], dtype=np.float32)
        if np.all(np.isfinite(xyz)):
            self.position = xyz

    def path_cb(self, msg):
        if not msg.poses:
            self.path = None
            return
        pts = np.array([
            [float(ps.pose.position.x), float(ps.pose.position.y), float(ps.pose.position.z)]
            for ps in msg.poses
        ], dtype=np.float32)
        if not np.all(np.isfinite(pts)):
            return
        self.path = pts
        self.path_frame = msg.header.frame_id or self.frame_id

    def upcoming_samples(self):
        if self.position is None or self.path is None or len(self.path) == 0:
            return None

        d = np.linalg.norm(self.path - self.position[None, :], axis=1)
        start = int(np.argmin(d))
        selected = [self.path[start]]
        total = 0.0
        for i in range(start + 1, len(self.path)):
            a = self.path[i - 1]
            b = self.path[i]
            seg = float(np.linalg.norm(b - a))
            if seg <= 1e-6:
                continue
            n = max(1, int(math.ceil(seg / self.sample_step_m)))
            for j in range(1, n + 1):
                t = j / n
                selected.append(a + t * (b - a))
            total += seg
            if total >= self.lookahead_m:
                break
        return np.asarray(selected, dtype=np.float32)

    def set_observation(self, raw_blocked, hits):
        if raw_blocked:
            self.block_count += 1
            self.clear_count = 0
            if not self.blocked and self.block_count >= self.block_confirm_scans:
                self.blocked = True
                self.get_logger().warn(
                    f'LOCAL PATH BLOCKED: LiDAR obstacle intersects upcoming corridor (hits={hits})')
        else:
            self.clear_count += 1
            self.block_count = 0
            if self.blocked and self.clear_count >= self.clear_confirm_scans:
                self.blocked = False
                self.get_logger().info('LOCAL PATH CLEAR again')

        out = Bool()
        out.data = bool(self.blocked)
        self.block_pub.publish(out)

    def cloud_cb(self, msg):
        if self.position is None or self.path is None:
            self.set_observation(False, 0)
            return

        cloud_frame = msg.header.frame_id or self.frame_id
        if self.path_frame and cloud_frame != self.path_frame:
            now = self.now_s()
            if now - self.last_frame_warning > 5.0:
                self.get_logger().warn(
                    f'Frame mismatch: cloud={cloud_frame}, active_path={self.path_frame}; skipping local check')
                self.last_frame_warning = now
            return

        try:
            pts = point_cloud2.read_points_numpy(
                msg, field_names=('x', 'y', 'z'), skip_nans=True)
            pts = np.asarray(pts)
            if pts.dtype.names:
                pts = np.column_stack([pts['x'], pts['y'], pts['z']])
            pts = pts.reshape(-1, 3).astype(np.float32, copy=False)
        except Exception as exc:
            self.get_logger().error(f'PointCloud2 conversion failed: {exc}')
            return

        if len(pts) == 0:
            self.set_observation(False, 0)
            return

        pts = pts[::self.point_stride]
        rel = pts - self.position[None, :]
        local_mask = np.sum(rel * rel, axis=1) <= self.local_radius_m ** 2
        pts = pts[local_mask]
        if len(pts) == 0:
            self.set_observation(False, 0)
            return

        samples = self.upcoming_samples()
        if samples is None or len(samples) == 0:
            self.set_observation(False, 0)
            return

        max_hits = 0
        h2 = self.horizontal_clearance ** 2
        # Chunked vectorized check keeps CPU/memory bounded.
        for i in range(0, len(samples), 24):
            s = samples[i:i + 24]
            dx = pts[:, None, 0] - s[None, :, 0]
            dy = pts[:, None, 1] - s[None, :, 1]
            dz = np.abs(pts[:, None, 2] - s[None, :, 2])
            inside = ((dx * dx + dy * dy) <= h2) & (dz <= self.vertical_clearance)
            if inside.size:
                hits_per_sample = np.count_nonzero(inside, axis=0)
                max_hits = max(max_hits, int(np.max(hits_per_sample)))
                if max_hits >= self.min_points:
                    break

        self.set_observation(max_hits >= self.min_points, max_hits)


def main(args=None):
    rclpy.init(args=args)
    node = LocalPathMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
