#!/usr/bin/env python3

import math

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleOdometry,
)

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


class DroneOffboardControl(Node):

    def __init__(self):
        super().__init__('drone_offboard_control')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # -------------------------
        # PX4 publishers
        # -------------------------

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

        # -------------------------
        # PX4 odometry subscriber
        # -------------------------

        self.odometry_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            qos
        )

        # Latest PX4 position estimate
        self.position = [0.0, 0.0, 0.0]
        self.have_odometry = False

        # Current PX4 yaw and the heading that will be held
        # throughout the Offboard mission.
        self.current_yaw = 0.0
        self.hold_yaw = None

        # Offboard initialization counter
        self.counter = 0

        # State machine
        self.state = 'INIT'
        self.state_start_time = self.get_clock().now()

        # Waypoints in PX4 NED coordinates
        self.takeoff_target = [0.0, 0.0, -0.85]
        self.waypoint_target = [1.0, 0.0, -0.85]
        self.return_target = [0.0, 0.0, -0.85]

        # Position tolerances
        self.horizontal_tolerance = 0.25
        self.vertical_tolerance = 0.20

        # 10 Hz control loop
        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )

        self.get_logger().info(
            'M2B-2 autonomous waypoint controller started'
        )

    # ================================================================
    # ODOMETRY
    # ================================================================

    def odometry_callback(self, msg):

        position = [
            float(msg.position[0]),
            float(msg.position[1]),
            float(msg.position[2])
        ]

        # PX4 VehicleOdometry quaternion order is:
        # [w, x, y, z]
        q = [
            float(msg.q[0]),
            float(msg.q[1]),
            float(msg.q[2]),
            float(msg.q[3])
        ]

# Ignore invalid odometry samples.
        if not all(math.isfinite(v) for v in position + q):
            return

        self.position = position

        w, x, y, z = q

        # Current heading in PX4's local NED frame.
        sin_yaw = 2.0 * (w * z + x * y)
        cos_yaw = 1.0 - 2.0 * (y * y + z * z)

        self.current_yaw = math.atan2(
            sin_yaw,
            cos_yaw
        )

        self.have_odometry = True

    # ================================================================
    # MAIN STATE MACHINE
    # ================================================================

    def timer_callback(self):

        # Once LAND mode has been requested,
        # stop sending Offboard trajectory commands.
        if self.state == 'LANDING':
            return

        # PX4 requires continuous Offboard heartbeat.
        self.publish_offboard_control_mode()

        # ------------------------------------------------------------
        # INIT
        # ------------------------------------------------------------

        if self.state == 'INIT':

            # Wait for valid PX4 odometry before starting Offboard.
            if not self.have_odometry:
                return

            # Capture the vehicle's current heading once.
            if self.hold_yaw is None:
                self.hold_yaw = self.current_yaw

                self.get_logger().info(
                    f'Captured initial yaw: '
                    f'{self.hold_yaw:.3f} rad '
                    f'({math.degrees(self.hold_yaw):.1f} deg)'
                )
# Send the takeoff position while preserving
            # the heading the drone had at startup.
            self.publish_position_setpoint(
                *self.takeoff_target,
                yaw=self.hold_yaw
            )

            # Send several setpoints before entering Offboard.
            if self.counter == 10:

                self.engage_offboard_mode()
                self.arm()

                self.change_state('TAKEOFF')

            if self.counter < 11:
                self.counter += 1

            return

        # ------------------------------------------------------------
        # TAKEOFF / FIRST HOLD
        # ------------------------------------------------------------

        if self.state in ['TAKEOFF', 'HOLD_TAKEOFF']:

            target = self.takeoff_target

        # ------------------------------------------------------------
        # WAYPOINT / WAYPOINT HOLD
        # ------------------------------------------------------------

        elif self.state in ['WAYPOINT', 'HOLD_WAYPOINT']:

            target = self.waypoint_target

        # ------------------------------------------------------------
        # RETURN / RETURN HOLD
        # ------------------------------------------------------------

        elif self.state in ['RETURN', 'HOLD_RETURN']:

            target = self.return_target

        else:
            return

        # Continuously command current target.
        self.publish_position_setpoint(
            *target,
            yaw=self.hold_yaw
        )

        # Don't make navigation decisions until PX4 odometry exists.
        if not self.have_odometry:
            return

        # ------------------------------------------------------------
        # TAKEOFF REACHED
        # ------------------------------------------------------------

        if self.state == 'TAKEOFF':

            if self.target_reached(self.takeoff_target):

                self.get_logger().info(
                    'Takeoff altitude reached: holding for 3 seconds'
                )

                self.change_state('HOLD_TAKEOFF')

        # ------------------------------------------------------------
        # HOLD AFTER TAKEOFF
        # ------------------------------------------------------------

        elif self.state == 'HOLD_TAKEOFF':

            if self.state_elapsed() >= 3.0:

                self.get_logger().info(
                    'Moving to waypoint: x=1.0, y=0.0, z=-0.85'
                )

                self.change_state('WAYPOINT')

        # ------------------------------------------------------------
        # WAYPOINT REACHED
        # ------------------------------------------------------------

        elif self.state == 'WAYPOINT':

            if self.target_reached(self.waypoint_target):

                self.get_logger().info(
                    'Waypoint reached: holding for 3 seconds'
                )

                self.change_state('HOLD_WAYPOINT')

        # ------------------------------------------------------------
        # HOLD AT WAYPOINT
        # ------------------------------------------------------------

        elif self.state == 'HOLD_WAYPOINT':

            if self.state_elapsed() >= 3.0:

                self.get_logger().info(
                    'Returning to takeoff position'
                )

                self.change_state('RETURN')

        # ------------------------------------------------------------
        # RETURN POSITION REACHED
        # ------------------------------------------------------------

        elif self.state == 'RETURN':

            if self.target_reached(self.return_target):

                self.get_logger().info(
                    'Return position reached: holding for 2 seconds'
                )

                self.change_state('HOLD_RETURN')

        # ------------------------------------------------------------
        # AUTOMATIC LANDING
        # ------------------------------------------------------------

        elif self.state == 'HOLD_RETURN':

            if self.state_elapsed() >= 2.0:

                self.get_logger().info(
                    'Flight sequence complete: LANDING'
                )

                self.land()

                self.change_state('LANDING')

    # ================================================================
    # STATE FUNCTIONS
    # ================================================================

    def change_state(self, new_state):

        self.state = new_state
        self.state_start_time = self.get_clock().now()

        self.get_logger().info(
            f'STATE -> {new_state}'
        )

    def state_elapsed(self):

        duration = (
            self.get_clock().now()
            - self.state_start_time
        )

        return duration.nanoseconds / 1e9

    # ================================================================
    # POSITION CHECKING
    # ================================================================

    def target_reached(self, target):

        dx = self.position[0] - target[0]
        dy = self.position[1] - target[1]
        dz = self.position[2] - target[2]

        horizontal_error = math.sqrt(
            dx * dx + dy * dy
        )

        vertical_error = abs(dz)

        return (
            horizontal_error <= self.horizontal_tolerance
            and
            vertical_error <= self.vertical_tolerance
        )

    # ================================================================
    # PX4 OFFBOARD COMMANDS
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
            self.get_clock().now().nanoseconds / 1000
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
            self.get_clock().now().nanoseconds / 1000
        )

        self.trajectory_pub.publish(msg)

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
            self.get_clock().now().nanoseconds / 1000
        )

        self.vehicle_command_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = DroneOffboardControl()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
