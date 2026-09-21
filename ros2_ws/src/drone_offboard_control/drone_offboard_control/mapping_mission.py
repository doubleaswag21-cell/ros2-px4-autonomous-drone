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

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleOdometry,
)


class MappingMission(Node):

    def __init__(self):
        super().__init__('mapping_mission')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ============================================================
        # PX4 PUBLISHERS
        # ============================================================

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos
        )

        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos
        )

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos
        )

        # ============================================================
        # PX4 ODOMETRY SUBSCRIBER
        # ============================================================

        self.odometry_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            qos
        )

        self.position = [0.0, 0.0, 0.0]
        self.have_odometry = False

        self.current_yaw = 0.0
        self.hold_yaw = None

        # ============================================================
        # M6E MAPPING MISSION
        #
        # PX4 local frame is NED:
        #
        # +X = North
        # +Y = East
        # -Z = Up
        #
        # Flight path:
        #
        #                  +Y
        #                  |
        #                  |
        #       -X ------- HOME ------- +X
        #                  |
        #                  |
        #                  -Y
        #
        # Every direction returns to HOME before moving to the next.
        # ============================================================

        self.takeoff_target = [
            0.0,
            0.0,
            -2.0
        ]

        self.mission_points = [

            (
                'PLUS_X',
                [2.0, 0.0, -2.0]
            ),

            (
                'CENTER',
                [0.0, 0.0, -2.0]
            ),

            (
                'PLUS_Y',
                [0.0, 2.0, -2.0]
            ),

            (
                'CENTER',
                [0.0, 0.0, -2.0]
            ),

            (
                'MINUS_X',
                [-2.0, 0.0, -2.0]
            ),

            (
                'CENTER',
                [0.0, 0.0, -2.0]
            ),

            (
                'MINUS_Y',
                [0.0, -2.0, -2.0]
            ),

            (
                'CENTER',
                [0.0, 0.0, -2.0]
            ),
        ]

        self.mission_index = 0

        # Position tolerances
        self.horizontal_tolerance = 0.25
        self.vertical_tolerance = 0.20

        # Hold durations
        self.takeoff_hold_seconds = 3.0
        self.point_hold_seconds = 2.0

        # Offboard initialization counter
        self.counter = 0

        # State machine
        self.state = 'INIT'
        self.state_start_time = self.get_clock().now()

        # 10 Hz controller loop
        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )

        self.get_logger().info(
            'M6E mapping mission controller started'
        )

    # ================================================================
    # PX4 ODOMETRY
    # ================================================================

    def odometry_callback(self, msg):

        position = [
            float(msg.position[0]),
            float(msg.position[1]),
            float(msg.position[2])
        ]

        # PX4 VehicleOdometry quaternion order:
        # [w, x, y, z]
        q = [
            float(msg.q[0]),
            float(msg.q[1]),
            float(msg.q[2]),
            float(msg.q[3])
        ]

        # Ignore invalid odometry samples.
        if not all(
            math.isfinite(value)
            for value in position + q
        ):
            return

        self.position = position

        w, x, y, z = q

        # Convert quaternion to yaw.
        sin_yaw = 2.0 * (
            w * z +
            x * y
        )

        cos_yaw = 1.0 - 2.0 * (
            y * y +
            z * z
        )

        self.current_yaw = math.atan2(
            sin_yaw,
            cos_yaw
        )

        self.have_odometry = True

    # ================================================================
    # MAIN STATE MACHINE
    # ================================================================

    def timer_callback(self):

        # Stop sending Offboard trajectory setpoints after LAND command.
        if self.state == 'LANDING':
            return

        # PX4 requires continuous Offboard heartbeat.
        self.publish_offboard_control_mode()

        # ------------------------------------------------------------
        # INIT
        # ------------------------------------------------------------

        if self.state == 'INIT':

            if not self.have_odometry:
                return

            # Capture current heading once.
            if self.hold_yaw is None:

                self.hold_yaw = self.current_yaw

                self.get_logger().info(
                    f'Captured initial yaw: '
                    f'{self.hold_yaw:.3f} rad '
                    f'({math.degrees(self.hold_yaw):.1f} deg)'
                )

            # Send takeoff setpoint before requesting Offboard.
            self.publish_position_setpoint(
                *self.takeoff_target,
                yaw=self.hold_yaw
            )

            # PX4 wants setpoints before entering Offboard.
            if self.counter == 10:

                self.engage_offboard_mode()
                self.arm()

                self.change_state(
                    'TAKEOFF'
                )

            if self.counter < 11:
                self.counter += 1

            return

        # ------------------------------------------------------------
        # TAKEOFF
        # ------------------------------------------------------------

        if self.state == 'TAKEOFF':

            self.publish_position_setpoint(
                *self.takeoff_target,
                yaw=self.hold_yaw
            )

            if self.target_reached(
                self.takeoff_target
            ):

                self.get_logger().info(
                    'Takeoff altitude reached: '
                    'holding for 3 seconds'
                )

                self.change_state(
                    'HOLD_TAKEOFF'
                )

            return

        # ------------------------------------------------------------
        # HOLD AFTER TAKEOFF
        # ------------------------------------------------------------

        if self.state == 'HOLD_TAKEOFF':

            self.publish_position_setpoint(
                *self.takeoff_target,
                yaw=self.hold_yaw
            )

            if (
                self.state_elapsed()
                >= self.takeoff_hold_seconds
            ):

                self.mission_index = 0

                name, target = (
                    self.mission_points[
                        self.mission_index
                    ]
                )

                self.get_logger().info(
                    f'Moving to {name}: '
                    f'x={target[0]:.1f}, '
                    f'y={target[1]:.1f}, '
                    f'z={target[2]:.1f}'
                )

                self.change_state(
                    'MOVING'
                )

            return

        # ------------------------------------------------------------
        # MOVE TO CURRENT MAPPING POINT
        # ------------------------------------------------------------

        if self.state == 'MOVING':

            name, target = (
                self.mission_points[
                    self.mission_index
                ]
            )

            self.publish_position_setpoint(
                *target,
                yaw=self.hold_yaw
            )

            if self.target_reached(target):

                self.get_logger().info(
                    f'{name} reached: '
                    f'holding for '
                    f'{self.point_hold_seconds:.0f} seconds'
                )

                self.change_state(
                    'HOLD_POINT'
                )

            return

        # ------------------------------------------------------------
        # HOLD CURRENT MAPPING POINT
        # ------------------------------------------------------------

        if self.state == 'HOLD_POINT':

            name, target = (
                self.mission_points[
                    self.mission_index
                ]
            )

            self.publish_position_setpoint(
                *target,
                yaw=self.hold_yaw
            )

            if (
                self.state_elapsed()
                >= self.point_hold_seconds
            ):

                self.mission_index += 1

                # ----------------------------------------------------
                # Mission finished
                # ----------------------------------------------------

                if (
                    self.mission_index
                    >= len(self.mission_points)
                ):

                    self.get_logger().info(
                        'Mapping trajectory complete: '
                        'LANDING'
                    )

                    self.land()

                    self.change_state(
                        'LANDING'
                    )

                    return

                # ----------------------------------------------------
                # Move to next mapping point
                # ----------------------------------------------------

                next_name, next_target = (
                    self.mission_points[
                        self.mission_index
                    ]
                )

                self.get_logger().info(
                    f'Moving to {next_name}: '
                    f'x={next_target[0]:.1f}, '
                    f'y={next_target[1]:.1f}, '
                    f'z={next_target[2]:.1f}'
                )

                self.change_state(
                    'MOVING'
                )

            return

    # ================================================================
    # STATE HELPERS
    # ================================================================

    def change_state(
        self,
        new_state
    ):

        self.state = new_state

        self.state_start_time = (
            self.get_clock().now()
        )

        self.get_logger().info(
            f'STATE -> {new_state}'
        )

    def state_elapsed(self):

        elapsed = (
            self.get_clock().now()
            -
            self.state_start_time
        )

        return (
            elapsed.nanoseconds
            /
            1e9
        )

    def target_reached(
        self,
        target
    ):

        if not self.have_odometry:
            return False

        dx = (
            self.position[0]
            -
            target[0]
        )

        dy = (
            self.position[1]
            -
            target[1]
        )

        dz = (
            self.position[2]
            -
            target[2]
        )

        horizontal_error = math.sqrt(
            dx * dx +
            dy * dy
        )

        vertical_error = abs(dz)

        return (
            horizontal_error
            <=
            self.horizontal_tolerance

            and

            vertical_error
            <=
            self.vertical_tolerance
        )

    # ================================================================
    # PX4 OFFBOARD CONTROL
    # ================================================================

    def publish_offboard_control_mode(self):

        msg = OffboardControlMode()

        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False

        msg.timestamp = int(
            self.get_clock()
            .now()
            .nanoseconds
            /
            1000
        )

        self.offboard_mode_pub.publish(msg)

    def publish_position_setpoint(
        self,
        x,
        y,
        z,
        yaw
    ):

        msg = TrajectorySetpoint()

        msg.position = [
            float(x),
            float(y),
            float(z)
        ]

        msg.yaw = float(yaw)

        msg.timestamp = int(
            self.get_clock()
            .now()
            .nanoseconds
            /
            1000
        )

        self.trajectory_pub.publish(msg)

    # ================================================================
    # PX4 VEHICLE COMMANDS
    # ================================================================

    def engage_offboard_mode(self):

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0
        )

        self.get_logger().info(
            'Offboard mode requested'
        )

    def arm(self):

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0
        )

        self.get_logger().info(
            'Arm command sent'
        )

    def land(self):

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND
        )

        self.get_logger().info(
            'Land command sent'
        )

    def publish_vehicle_command(
        self,
        command,
        param1=0.0,
        param2=0.0
    ):

        msg = VehicleCommand()

        msg.command = command

        msg.param1 = float(param1)
        msg.param2 = float(param2)

        msg.target_system = 1
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        msg.timestamp = int(
            self.get_clock()
            .now()
            .nanoseconds
            /
            1000
        )

        self.vehicle_command_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = MappingMission()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
