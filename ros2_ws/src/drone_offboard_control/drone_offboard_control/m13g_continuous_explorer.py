#!/usr/bin/env python3
# Copyright (c) 2026 Ammaar Ahmed
# Project-specific ROS 2 autonomy implementation.

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool, Empty
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry, VehicleStatus


class M13GContinuousExplorer(Node):
    ARMING_STATE_ARMED = 2
    NAVIGATION_STATE_OFFBOARD = 14

    def __init__(self):
        super().__init__('m13g_continuous_explorer')

        self.flight_enable = bool(self.declare_parameter('flight_enable', False).value)
        self.h_tol = float(self.declare_parameter('horizontal_tolerance', 0.20).value)
        self.v_tol = float(self.declare_parameter('vertical_tolerance', 0.15).value)
        self.start_align_max = float(self.declare_parameter('start_alignment_max', 0.80).value)
        self.start_vertical_align_max = float(self.declare_parameter('start_vertical_alignment_max', 0.20).value)
        self.min_new_goal_distance = float(self.declare_parameter('min_new_goal_distance', 0.60).value)
        self.min_hold_sec = float(self.declare_parameter('min_hold_before_new_path_sec', 1.0).value)
        self.no_new_path_timeout = float(self.declare_parameter('no_new_path_timeout_sec', 30.0).value)
        self.wp_timeout = float(self.declare_parameter('waypoint_timeout_sec', 90.0).value)
        self.odom_timeout = float(self.declare_parameter('odom_timeout_sec', 1.0).value)
        self.max_cycles = int(self.declare_parameter('max_cycles', 30).value)
        self.takeoff_altitude = float(self.declare_parameter('takeoff_altitude', 1.55).value)
        self.first_path_timeout = float(self.declare_parameter('first_path_timeout_sec', 30.0).value)
        self.replan_timeout = float(self.declare_parameter('replan_timeout_sec', 30.0).value)
        self.replan_topic = str(self.declare_parameter('replan_topic', '/replan_requested').value)
        self.blocked_topic = str(self.declare_parameter('blocked_topic', '/local_path_blocked').value)
        self.reject_topic = str(self.declare_parameter('reject_topic', '/path_rejected').value)
        self.map_odom_topic = str(self.declare_parameter('map_odom_topic', '/aft_mapped_to_init').value)
        self.map_odom_timeout = float(self.declare_parameter('map_odom_timeout_sec', 1.0).value)

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        path_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', px4_qos)
        self.replan_pub = self.create_publisher(Empty, self.replan_topic, 10)
        self.reject_pub = self.create_publisher(Empty, self.reject_topic, 10)
        self.active_path_pub = self.create_publisher(Path, '/m13g/active_path', path_qos)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_cb, px4_qos)
        self.create_subscription(Odometry, self.map_odom_topic, self.map_odom_cb, px4_qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.status_cb, px4_qos)
        self.create_subscription(Path, '/planned_path', self.path_cb, path_qos)
        self.create_subscription(Bool, self.blocked_topic, self.blocked_cb, 10)

        self.position = [0.0, 0.0, 0.0]
        self.yaw = 0.0
        self.have_odom = False
        self.last_odom_time = None
        self.vehicle_status = None
        self.map_position_enu = None
        self.last_map_odom_time = None

        self.state = 'WAIT_READY'
        self.state_start = self.now_s()
        self.active_path = None
        self.active_sig = None
        self.wp_index = 0
        self.wp_start = None
        self.hold_target = None
        self.last_completed_goal = None
        self.completed_cycles = 0
        self.last_mode_request = 0.0
        self.last_arm_request = 0.0
        self.last_land_request = 0.0
        self.replan_sent_for_state = False
        self.local_path_blocked = False

        self.create_timer(0.1, self.tick)

        self.get_logger().info('========================================')
        self.get_logger().info('M13G CONTINUOUS FRONTIER EXPLORER')
        self.get_logger().info('Bootstrap takeoff -> repeated live A* paths -> final land')
        self.get_logger().info(f'Bootstrap altitude: {self.takeoff_altitude:.2f} m')
        self.get_logger().info(f'No-new-path timeout: {self.no_new_path_timeout:.1f} s')
        self.get_logger().info(f'Maximum cycles: {self.max_cycles}')
        if self.flight_enable:
            self.get_logger().warn('FLIGHT ENABLED')
        else:
            self.get_logger().warn('FLIGHT DISABLED')
        self.get_logger().info('Waiting for path + PX4 odometry/status...')
        self.get_logger().info('========================================')

    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def elapsed(self):
        return self.now_s() - self.state_start

    def odom_cb(self, msg):
        p = [float(msg.position[0]), float(msg.position[1]), float(msg.position[2])]
        if not all(math.isfinite(v) for v in p):
            return
        self.position = p
        self.have_odom = True
        self.last_odom_time = self.now_s()
        q = msg.q
        if len(q) >= 4:
            w, x, y, z = [float(v) for v in q[:4]]
            if all(math.isfinite(v) for v in (w, x, y, z)):
                self.yaw = math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))

    def map_odom_cb(self, msg):
        p = msg.pose.pose.position
        xyz = [float(p.x), float(p.y), float(p.z)]
        if not all(math.isfinite(v) for v in xyz):
            return
        self.map_position_enu = xyz
        self.last_map_odom_time = self.now_s()

    def status_cb(self, msg):
        self.vehicle_status = msg

    @staticmethod
    def enu_to_ned(x, y, z):
        return [float(y), float(x), float(-z)]

    @staticmethod
    def hdist(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def convert_path(self, msg):
        # /planned_path is in Point-LIO camera_init (ROS ENU). PX4 odometry
        # is in PX4's own local NED frame. Axis swapping alone is not enough:
        # their local origins can differ after estimator/session resets.
        # Align the two frames at the vehicle's current pose, then apply the
        # same translation to every path waypoint. This preserves the A* path
        # geometry and intentional safe-start snap.
        if self.map_position_enu is None or not self.have_odom:
            self.get_logger().warn('Path received before map/PX4 odometry alignment is ready')
            return None

        if (self.last_map_odom_time is None or
                self.now_s() - self.last_map_odom_time > self.map_odom_timeout):
            self.get_logger().warn('Path received with stale Point-LIO odometry; rejecting')
            return None

        map_vehicle_ned = self.enu_to_ned(*self.map_position_enu)
        frame_offset = [
            self.position[i] - map_vehicle_ned[i]
            for i in range(3)
        ]

        out = []
        for ps in msg.poses:
            p = ps.pose.position
            xyz = [float(p.x), float(p.y), float(p.z)]
            if not all(math.isfinite(v) for v in xyz):
                return None
            raw_ned = self.enu_to_ned(*xyz)
            out.append([raw_ned[i] + frame_offset[i] for i in range(3)])

        self.get_logger().info(
            f'MAP->PX4 NED offset: [{frame_offset[0]:.2f}, '
            f'{frame_offset[1]:.2f}, {frame_offset[2]:.2f}] m')
        return out

    @staticmethod
    def signature(path):
        f = path[-1]
        return (len(path), round(f[0], 2), round(f[1], 2), round(f[2], 2))

    def validate_path(self, path, require_new_goal):
        if not path:
            return False, 'empty path'
        err = self.hdist(self.position, path[0])
        if err > self.start_align_max:
            return False, f'start horizontal error {err:.2f} m > {self.start_align_max:.2f} m'
        vertical_err = abs(self.position[2] - path[0][2])
        if vertical_err > self.start_vertical_align_max:
            return False, (
                f'start vertical error {vertical_err:.2f} m > '
                f'{self.start_vertical_align_max:.2f} m')
        if require_new_goal and self.last_completed_goal is not None:
            shift = self.hdist(path[-1], self.last_completed_goal)
            if shift < self.min_new_goal_distance:
                return False, f'goal shift only {shift:.2f} m'
        return True, ''

    def load_path(self, path, label, source_msg=None):
        self.active_path = path
        self.active_sig = self.signature(path)
        self.wp_index = 0
        self.wp_start = self.now_s()
        if source_msg is not None:
            self.active_path_pub.publish(source_msg)
        self.get_logger().info('========================================')
        self.get_logger().info(f'{label}: {len(path)} waypoints')
        self.get_logger().info(f'Current NED: [{self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f}]')
        self.get_logger().info(f'First NED:   [{path[0][0]:.2f}, {path[0][1]:.2f}, {path[0][2]:.2f}]')
        self.get_logger().info(f'Final NED:   [{path[-1][0]:.2f}, {path[-1][1]:.2f}, {path[-1][2]:.2f}]')
        self.get_logger().info(f'Start horizontal error: {self.hdist(self.position, path[0]):.3f} m')
        self.get_logger().info(f'Start vertical error: {abs(self.position[2] - path[0][2]):.3f} m')
        self.get_logger().info('========================================')

    def request_replan(self, reason):
        self.replan_pub.publish(Empty())
        self.replan_sent_for_state = True
        self.get_logger().info(f'REPLAN REQUESTED: {reason}')

    def reject_planner_path(self, reason):
        self.reject_pub.publish(Empty())
        self.get_logger().warn(f'PATH REJECTED -> trying another frontier: {reason}')

    def blocked_cb(self, msg):
        self.local_path_blocked = bool(msg.data)
        if self.local_path_blocked and self.state == 'FOLLOW_PATH':
            self.hold_target = list(self.position)
            self.get_logger().warn('LOCAL OBSTACLE ON ACTIVE PATH -> HOLD + IMMEDIATE REPLAN')
            self.change_state('WAIT_REPLAN')
            self.request_replan('local path blocked')

    def path_cb(self, msg):
        path = self.convert_path(msg)
        if not path:
            return
        sig = self.signature(path)

        # Bootstrap vertically first; no initial path is required on the ground.
        if self.state in ('WAIT_READY', 'PRESTREAM', 'WAIT_OFFBOARD', 'WAIT_ARM', 'TAKEOFF'):
            return

        if self.state == 'WAIT_FIRST_PATH':
            ok, reason = self.validate_path(path, False)
            if not ok:
                self.get_logger().warn(f'First airborne path rejected: {reason}')
                self.reject_planner_path(reason)
                return
            if sig != self.active_sig:
                self.load_path(path, 'FIRST AIRBORNE FRONTIER PATH', msg)
                self.change_state('FOLLOW_PATH')
            return

        if self.state == 'WAIT_REPLAN':
            ok, reason = self.validate_path(path, False)
            if not ok:
                self.get_logger().warn(f'Recovery path rejected: {reason}')
                self.reject_planner_path(reason)
                return
            self.load_path(path, f'RECOVERY PATH #{self.completed_cycles + 1}', msg)
            self.change_state('FOLLOW_PATH')
            return

        # Normal periodic replans are ignored during traversal. Only the local
        # safety monitor is allowed to interrupt FOLLOW_PATH.
        if self.state == 'FOLLOW_PATH':
            return

        if self.state == 'HOLD_FOR_NEXT':
            if self.elapsed() < self.min_hold_sec or sig == self.active_sig:
                return
            ok, reason = self.validate_path(path, True)
            if not ok:
                self.get_logger().warn(f'Next frontier path rejected: {reason}')
                self.reject_planner_path(reason)
                return
            self.load_path(path, f'NEW FRONTIER PATH #{self.completed_cycles + 1}', msg)
            self.change_state('FOLLOW_PATH')

    def tick(self):
        now = self.now_s()

        if (self.state not in ('WAIT_READY', 'LANDING', 'COMPLETE') and
                self.last_odom_time is not None and
                now - self.last_odom_time > self.odom_timeout):
            self.abort(f'odometry stale for {now - self.last_odom_time:.2f} s')
            return

        if self.state == 'WAIT_READY':
            if self.flight_enable and self.have_odom and self.vehicle_status is not None:
                self.hold_target = [
                    float(self.position[0]),
                    float(self.position[1]),
                    float(-self.takeoff_altitude),
                ]
                self.get_logger().info(
                    f'BOOTSTRAP target NED: [{self.hold_target[0]:.2f}, '
                    f'{self.hold_target[1]:.2f}, {self.hold_target[2]:.2f}]')
                self.get_logger().info(
                    'No initial frontier/path required on the ground')
                self.change_state('PRESTREAM')
            return

        if self.state in ('PRESTREAM', 'WAIT_OFFBOARD', 'WAIT_ARM', 'TAKEOFF', 'WAIT_FIRST_PATH', 'FOLLOW_PATH', 'HOLD_FOR_NEXT', 'WAIT_REPLAN'):
            target = self.current_target()
            if target is not None:
                self.publish_offboard()
                self.publish_setpoint(*target, self.yaw)

        if self.state == 'PRESTREAM':
            if self.elapsed() >= 4.0:
                self.request_offboard()
                self.last_mode_request = now
                self.change_state('WAIT_OFFBOARD')
            return

        if self.state == 'WAIT_OFFBOARD':
            if self.is_offboard():
                self.get_logger().info('PX4 entered OFFBOARD')
                self.arm()
                self.last_arm_request = now
                self.change_state('WAIT_ARM')
            elif now - self.last_mode_request >= 1.0:
                self.request_offboard()
                self.last_mode_request = now
            return

        if self.state == 'WAIT_ARM':
            if self.is_armed():
                self.get_logger().info('ARMED - BOOTSTRAP TAKEOFF STARTED')
                self.change_state('TAKEOFF')
            elif now - self.last_arm_request >= 1.0:
                self.arm()
                self.last_arm_request = now
            return

        if self.state == 'TAKEOFF':
            if not self.is_armed():
                self.abort('vehicle disarmed during bootstrap takeoff')
                return
            if not self.is_offboard():
                self.abort('vehicle left OFFBOARD during bootstrap takeoff')
                return
            if self.reached(self.hold_target):
                self.get_logger().info(
                    f'BOOTSTRAP TAKEOFF COMPLETE at {self.takeoff_altitude:.2f} m')
                self.get_logger().info(
                    'Holding for Point-LIO/OctoMap/M13C/M13D to produce first path...')
                self.change_state('WAIT_FIRST_PATH')
                self.request_replan('bootstrap takeoff complete')
            return

        if self.state == 'WAIT_FIRST_PATH':
            if not self.is_armed():
                self.abort('vehicle disarmed while waiting for first path')
                return
            if not self.is_offboard():
                self.abort('vehicle left OFFBOARD while waiting for first path')
                return
            if self.elapsed() >= self.first_path_timeout:
                self.get_logger().warn(
                    f'No valid first frontier path for {self.first_path_timeout:.1f} s')
                self.get_logger().warn(
                    'Bootstrap exploration stalled -> LAND')
                self.begin_landing()
            return

        if self.state == 'WAIT_REPLAN':
            if not self.is_armed():
                self.abort('vehicle disarmed while waiting for recovery path')
                return
            if not self.is_offboard():
                self.abort('vehicle left OFFBOARD while waiting for recovery path')
                return
            if self.elapsed() >= self.replan_timeout:
                self.get_logger().warn(f'No valid recovery path for {self.replan_timeout:.1f} s -> LAND')
                self.begin_landing()
            return

        if self.state == 'FOLLOW_PATH':
            if not self.is_armed():
                self.abort('vehicle disarmed during path following')
                return
            if not self.is_offboard():
                self.abort('vehicle left OFFBOARD during path following')
                return
            if self.active_path is None:
                self.abort('no active path')
                return

            target = self.active_path[self.wp_index]
            if self.reached(target):
                self.get_logger().info(f'Cycle {self.completed_cycles + 1} WP {self.wp_index:02d} REACHED')
                self.wp_index += 1
                self.wp_start = now

                if self.wp_index >= len(self.active_path):
                    self.completed_cycles += 1
                    self.last_completed_goal = list(self.active_path[-1])
                    self.hold_target = list(self.active_path[-1])
                    self.get_logger().info('========================================')
                    self.get_logger().info(f'M13G FRONTIER CYCLE {self.completed_cycles} COMPLETE')
                    self.get_logger().info('HOLDING FOR MAP GROWTH + NEW FRONTIER')
                    self.get_logger().info('========================================')
                    if self.completed_cycles >= self.max_cycles:
                        self.get_logger().warn('Maximum cycle limit reached')
                        self.begin_landing()
                    else:
                        self.change_state('HOLD_FOR_NEXT')
                    return

                nxt = self.active_path[self.wp_index]
                self.get_logger().info(f'Next WP {self.wp_index:02d}: [{nxt[0]:.2f}, {nxt[1]:.2f}, {nxt[2]:.2f}]')
            elif self.wp_start is not None and now - self.wp_start > self.wp_timeout:
                self.abort(f'waypoint {self.wp_index} timeout')
            return

        if self.state == 'HOLD_FOR_NEXT':
            if not self.is_armed():
                self.change_state('COMPLETE')
                return
            if not self.is_offboard():
                self.abort('left OFFBOARD while waiting for next path')
                return
            if self.elapsed() >= self.min_hold_sec and not self.replan_sent_for_state:
                self.request_replan('frontier cycle complete')
            if self.elapsed() >= self.no_new_path_timeout:
                self.get_logger().warn(f'No distinct valid path for {self.no_new_path_timeout:.1f} s')
                self.get_logger().warn('Exploration complete/stalled -> LAND')
                self.begin_landing()
            return

        if self.state == 'LANDING':
            if not self.is_armed():
                self.get_logger().info('Vehicle disarmed')
                self.get_logger().info('M13G CONTINUOUS EXPLORATION COMPLETE')
                self.get_logger().info(f'Completed frontier cycles: {self.completed_cycles}')
                self.change_state('COMPLETE')
            elif now - self.last_land_request >= 4.0:
                self.land()
                self.last_land_request = now

    def current_target(self):
        if self.state == 'FOLLOW_PATH' and self.active_path is not None and self.wp_index < len(self.active_path):
            return self.active_path[self.wp_index]
        return self.hold_target

    def reached(self, target):
        dx = self.position[0] - target[0]
        dy = self.position[1] - target[1]
        dz = self.position[2] - target[2]
        return math.hypot(dx, dy) <= self.h_tol and abs(dz) <= self.v_tol

    def publish_offboard(self):
        msg = OffboardControlMode()
        msg.timestamp = self.now_us()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self.offboard_pub.publish(msg)

    def publish_setpoint(self, x, y, z, yaw):
        msg = TrajectorySetpoint()
        msg.timestamp = self.now_us()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        self.trajectory_pub.publish(msg)

    def vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.now_us()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def request_offboard(self):
        self.vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info('OFFBOARD requested')

    def arm(self):
        self.vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
        self.get_logger().info('ARM requested')

    def land(self):
        self.vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().warn('LAND requested')

    def begin_landing(self):
        self.land()
        self.last_land_request = self.now_s()
        self.change_state('LANDING')

    def is_armed(self):
        return self.vehicle_status is not None and self.vehicle_status.arming_state == self.ARMING_STATE_ARMED

    def is_offboard(self):
        return self.vehicle_status is not None and self.vehicle_status.nav_state == self.NAVIGATION_STATE_OFFBOARD

    def abort(self, reason):
        if self.state in ('LANDING', 'COMPLETE'):
            return
        self.get_logger().error(f'M13G ABORT: {reason}')
        if self.is_armed():
            self.begin_landing()
        else:
            self.change_state('COMPLETE')

    def change_state(self, new_state):
        if new_state == self.state:
            return
        self.get_logger().info(f'STATE: {self.state} -> {new_state}')
        self.state = new_state
        self.state_start = self.now_s()
        self.replan_sent_for_state = False


def main(args=None):
    rclpy.init(args=args)
    node = M13GContinuousExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
