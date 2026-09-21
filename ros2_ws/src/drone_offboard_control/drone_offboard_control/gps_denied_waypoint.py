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
    VehicleStatus,
)


class GPSDeniedWaypoint(Node):

    # PX4 values confirmed from our current SITL build
    ARMING_STATE_ARMED = 2
    NAVIGATION_STATE_OFFBOARD = 14

    def __init__(self):
        super().__init__('gps_denied_waypoint')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ----------------------------------------------------------
        # PX4 publishers
        # ----------------------------------------------------------

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            px4_qos,
        )

        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            px4_qos,
        )

        self.command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            px4_qos,
        )

        # ----------------------------------------------------------
        # PX4 subscribers
        # ----------------------------------------------------------

        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            px4_qos,
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.status_callback,
            px4_qos,
        )

        # ----------------------------------------------------------
        # Mission settings
        # ----------------------------------------------------------

        self.takeoff_height = 0.85
        self.waypoint_distance = 1.0

        self.horizontal_tolerance = 0.20
        self.vertical_tolerance = 0.12

        # Safety limits relative to startup location
        self.max_horizontal_radius = 2.5
        self.max_height = 1.40

        # ----------------------------------------------------------
        # Vehicle state
        # ----------------------------------------------------------

        self.position = [0.0, 0.0, 0.0]
        self.yaw = 0.0

        self.have_odometry = False
        self.last_odom_time = None

        self.vehicle_status = None

        self.start_position = None
        self.hold_yaw = None

        self.takeoff_target = None
        self.waypoint_target = None
        self.return_target = None


        # ----------------------------------------------------------
        # State machine
        # ----------------------------------------------------------

        self.state = 'WAIT_ODOMETRY'
        self.state_start_time = self.get_clock().now()

        self.valid_odom_cycles = 0
        self.prestream_counter = 0

        self.last_mode_request = 0.0
        self.last_arm_request = 0.0
        self.last_land_request = 0.0

        self.arm_attempts = 0

        # 10 Hz controller
        self.timer = self.create_timer(
            0.1,
            self.timer_callback,
        )

        self.get_logger().info(
            'M7I GPS-denied waypoint controller started'
        )

        self.get_logger().info(
            'Waiting for stable PX4 local odometry...'
        )

    # ==============================================================
    # TIME
    # ==============================================================

    def now_us(self):
        return int(
            self.get_clock().now().nanoseconds / 1000
        )

    def now_seconds(self):
        return (
            self.get_clock().now().nanoseconds
            / 1_000_000_000.0
        )

    def state_elapsed(self):
        return (
            self.get_clock().now()
            - self.state_start_time
        ).nanoseconds / 1_000_000_000.0

    # ==============================================================
    # PX4 DATA
    # ==============================================================

    def odom_callback(self, msg):

        x = float(msg.position[0])
        y = float(msg.position[1])
        z = float(msg.position[2])

        if not all(math.isfinite(v) for v in [x, y, z]):
            return

        self.position = [x, y, z]
        self.last_odom_time = self.now_seconds()
        self.have_odometry = True

        # PX4 quaternion order: [w, x, y, z]
        q = msg.q

        if all(math.isfinite(float(v)) for v in q):

            w = float(q[0])
            qx = float(q[1])
            qy = float(q[2])
            qz = float(q[3])

            sin_yaw = 2.0 * (
                w * qz + qx * qy
            )

            cos_yaw = 1.0 - 2.0 * (
                qy * qy + qz * qz
            )

            self.yaw = math.atan2(
                sin_yaw,
                cos_yaw,
            )

    def status_callback(self, msg):
        self.vehicle_status = msg

    # ==============================================================
    # MAIN LOOP
    # ==============================================================

    def timer_callback(self):

        # ----------------------------------------------------------
        # WAIT FOR HEALTHY ODOMETRY
        # ----------------------------------------------------------

        if self.state == 'WAIT_ODOMETRY':

            if not self.have_odometry:
                return

            age = (
                self.now_seconds()
                - self.last_odom_time
            )

            if age > 0.5:
                self.valid_odom_cycles = 0
                return

            self.valid_odom_cycles += 1

            if self.valid_odom_cycles >= 20:

                self.get_logger().info(
                    'Stable PX4 odometry detected'
                )

                self.change_state('PRESTREAM')

            return

        # ----------------------------------------------------------
        # GENERAL ODOMETRY SAFETY
        # ----------------------------------------------------------

        if not self.have_odometry:
            self.trigger_abort(
                'PX4 odometry unavailable'
            )
            return

        odom_age = (
            self.now_seconds()
            - self.last_odom_time
        )

        if odom_age > 0.5:
            self.trigger_abort(
                f'PX4 odometry stale: '
                f'{odom_age:.2f} s'
            )
            return

        # ----------------------------------------------------------
        # PRE-ARM:
        # Hold whatever PX4 currently considers its position.
        # ----------------------------------------------------------

        if self.state in [
            'PRESTREAM',
            'WAIT_OFFBOARD',
            'WAIT_ARM',
        ]:

            target = self.position
            yaw = self.yaw

            self.publish_offboard_control_mode()

            self.publish_position_setpoint(
                target[0],
                target[1],
                target[2],
                yaw,
            )

        # ----------------------------------------------------------
        # MISSION:
        # Continue publishing active target.
        # ----------------------------------------------------------

        elif self.state in [
            'TAKEOFF',
            'HOLD_TAKEOFF',
        ]:

            self.publish_offboard_control_mode()

            self.publish_target(
                self.takeoff_target
            )

        elif self.state in [
            'WAYPOINT',
            'HOLD_WAYPOINT',
        ]:

            self.publish_offboard_control_mode()

            self.publish_target(
                self.waypoint_target
            )

        elif self.state in [
            'RETURN',
            'HOLD_RETURN',
        ]:

            self.publish_offboard_control_mode()

            self.publish_target(
                self.return_target
            )

        # ----------------------------------------------------------
        # PRESTREAM HEARTBEAT FOR 3 SECONDS
        # ----------------------------------------------------------

        if self.state == 'PRESTREAM':

            self.prestream_counter += 1

            if self.prestream_counter >= 30:

                self.get_logger().info(
                    'Offboard heartbeat established'
                )

                self.engage_offboard_mode()

                self.last_mode_request = (
                    self.now_seconds()
                )

                self.change_state(
                    'WAIT_OFFBOARD'
                )

            return

        # ----------------------------------------------------------
        # WAIT UNTIL PX4 ACTUALLY ENTERS OFFBOARD
        # ----------------------------------------------------------

        if self.state == 'WAIT_OFFBOARD':

            if (
                self.vehicle_status is not None
                and self.vehicle_status.nav_state
                == self.NAVIGATION_STATE_OFFBOARD
            ):

                self.get_logger().info(
                    'PX4 entered OFFBOARD'
                )

                self.arm()

                self.arm_attempts = 1
                self.last_arm_request = (
                    self.now_seconds()
                )

                self.change_state(
                    'WAIT_ARM'
                )

                return

            if (
                self.now_seconds()
                - self.last_mode_request
                >= 1.0
            ):

                self.engage_offboard_mode()

                self.last_mode_request = (
                    self.now_seconds()
                )

            return

        # ----------------------------------------------------------
        # WAIT UNTIL PX4 IS ACTUALLY ARMED
        # ----------------------------------------------------------

        if self.state == 'WAIT_ARM':

            if (
                self.vehicle_status is not None
                and self.vehicle_status.arming_state
                == self.ARMING_STATE_ARMED
            ):

                # ----------------------------------------------
                # CRITICAL:
                # Capture local origin AFTER arming.
                # Everything is now relative to THIS position.
                # ----------------------------------------------

                x0 = self.position[0]
                y0 = self.position[1]
                z0 = self.position[2]


                self.start_position = [
                    x0,
                    y0,
                    z0,
                ]

                self.hold_yaw = self.yaw

                self.takeoff_target = [
                    x0,
                    y0,
                    z0 - self.takeoff_height,
                ]

                self.waypoint_target = [
                    x0 + self.waypoint_distance,
                    y0,
                    z0 - self.takeoff_height,
                ]

                self.return_target = [
                    x0,
                    y0,
                    z0 - self.takeoff_height,
                ]

                self.get_logger().info(
                    'PX4 ARMED'
                )

                self.get_logger().info(
                    f'Start NED: '
                    f'[{x0:.3f}, '
                    f'{y0:.3f}, '
                    f'{z0:.3f}]'
                )

                self.get_logger().info(
                    f'Takeoff target: '
                    f'{self.takeoff_target}'
                )

                self.get_logger().info(
                    f'Waypoint target: '
                    f'{self.waypoint_target}'
                )

                self.change_state('TAKEOFF')

                return

            if (
                self.now_seconds()
                - self.last_arm_request
                >= 1.0
            ):

                if self.arm_attempts >= 5:

                    self.get_logger().error(
                        'Unable to arm after '
                        '5 attempts'
                    )

                    self.change_state(
                        'COMPLETE'
                    )

                    return

                self.arm()

                self.arm_attempts += 1

                self.last_arm_request = (
                    self.now_seconds()
                )

            return

        # ----------------------------------------------------------
        # SAFETY CHECKS WHILE AIRBORNE
        # ----------------------------------------------------------

        if self.start_position is not None:

            dx = (
                self.position[0]
                - self.start_position[0]
            )

            dy = (
                self.position[1]
                - self.start_position[1]
            )

            horizontal_radius = math.sqrt(
                dx * dx + dy * dy
            )

            height = (
                self.start_position[2]
                - self.position[2]
            )

            if (
                horizontal_radius
                > self.max_horizontal_radius
            ):

                self.trigger_abort(
                    'Horizontal safety radius exceeded'
                )

                return

            if height > self.max_height:

                self.trigger_abort(
                    'Maximum safe height exceeded'
                )

                return

        # ----------------------------------------------------------
        # TAKEOFF
        # ----------------------------------------------------------

        if self.state == 'TAKEOFF':

            if self.target_reached(
                self.takeoff_target
            ):

                self.get_logger().info(
                    'Takeoff target reached'
                )

                self.change_state(
                    'HOLD_TAKEOFF'
                )

        # ----------------------------------------------------------
        # HOLD AFTER TAKEOFF
        # ----------------------------------------------------------

        elif self.state == 'HOLD_TAKEOFF':

            if self.state_elapsed() >= 3.0:

                self.get_logger().info(
                    'Flying GPS-denied waypoint'
                )

                self.change_state(
                    'WAYPOINT'
                )

        # ----------------------------------------------------------
        # WAYPOINT
        # ----------------------------------------------------------

        elif self.state == 'WAYPOINT':

            if self.target_reached(
                self.waypoint_target
            ):

                self.get_logger().info(
                    'Waypoint reached'
                )

                self.change_state(
                    'HOLD_WAYPOINT'
                )

        # ----------------------------------------------------------
        # HOLD WAYPOINT
        # ----------------------------------------------------------

        elif self.state == 'HOLD_WAYPOINT':

            if self.state_elapsed() >= 3.0:

                self.get_logger().info(
                    'Returning to origin'
                )

                self.change_state(
                    'RETURN'
                )

        # ----------------------------------------------------------
        # RETURN
        # ----------------------------------------------------------

        elif self.state == 'RETURN':

            if self.target_reached(
                self.return_target
            ):

                self.get_logger().info(
                    'Return target reached'
                )

                self.change_state(
                    'HOLD_RETURN'
                )

        # ----------------------------------------------------------
        # LAND
        # ----------------------------------------------------------

        elif self.state == 'HOLD_RETURN':

            if self.state_elapsed() >= 2.0:

                self.get_logger().info(
                    'Mission complete: LAND'
                )

                self.land()

                self.last_land_request = (
                    self.now_seconds()
                )

                self.change_state(
                    'LANDING'
                )

        elif self.state == 'LANDING':

            if (
                self.vehicle_status is not None
                and self.vehicle_status.arming_state
                != self.ARMING_STATE_ARMED
            ):

                self.get_logger().info(
                    'Vehicle disarmed - '
                    'M7I complete'
                )

                self.change_state(
                    'COMPLETE'
                )

            elif (
                self.now_seconds()
                - self.last_land_request
                >= 2.0
            ):

                self.land()

                self.last_land_request = (
                    self.now_seconds()
                )

        elif self.state == 'ABORT':

            if (
                self.vehicle_status is not None
                and self.vehicle_status.arming_state
                == self.ARMING_STATE_ARMED
            ):

                if (
                    self.now_seconds()
                    - self.last_land_request
                    >= 1.0
                ):

                    self.land()

                    self.last_land_request = (
                        self.now_seconds()
                    )

            else:

                self.get_logger().warn(
                    'Abort complete - vehicle disarmed'
                )

                self.change_state(
                    'COMPLETE'
                )

    # ==============================================================
    # HELPERS
    # ==============================================================

    def publish_target(self, target):

        if target is None:
            return

        self.publish_position_setpoint(
            target[0],
            target[1],
            target[2],
            self.hold_yaw,
        )

    def target_reached(self, target):

        if target is None:
            return False

        dx = self.position[0] - target[0]
        dy = self.position[1] - target[1]
        dz = self.position[2] - target[2]

        horizontal_error = math.sqrt(
            dx * dx + dy * dy
        )

        return (
            horizontal_error
            <= self.horizontal_tolerance
            and abs(dz)
            <= self.vertical_tolerance
        )

    def publish_offboard_control_mode(self):

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

    def publish_position_setpoint(
        self,
        x,
        y,
        z,
        yaw,
    ):

        msg = TrajectorySetpoint()

        msg.timestamp = self.now_us()

        msg.position = [
            float(x),
            float(y),
            float(z),
        ]

        nan = float('nan')

        msg.velocity = [
            nan,
            nan,
            nan,
        ]

        msg.acceleration = [
            nan,
            nan,
            nan,
        ]

        msg.jerk = [
            nan,
            nan,
            nan,
        ]

        msg.yaw = float(yaw)
        msg.yawspeed = nan

        self.trajectory_pub.publish(msg)

    # ==============================================================
    # VEHICLE COMMANDS
    # ==============================================================

    def publish_vehicle_command(
        self,
        command,
        param1=0.0,
        param2=0.0,
    ):

        msg = VehicleCommand()

        msg.timestamp = self.now_us()

        msg.param1 = float(param1)
        msg.param2 = float(param2)

        msg.command = command

        msg.target_system = 1
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        self.command_pub.publish(msg)

    def engage_offboard_mode(self):

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1.0,
            6.0,
        )

        self.get_logger().info(
            'Offboard mode requested'
        )

    def arm(self):

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0,
            0.0,
        )

        self.get_logger().info(
            'ARM requested'
        )

    def land(self):

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND,
        )

        self.get_logger().warn(
            'LAND command sent'
        )

    # ==============================================================
    # SAFETY
    # ==============================================================

    def trigger_abort(self, reason):

        if self.state in [
            'ABORT',
            'LANDING',
            'COMPLETE',
        ]:
            return

        self.get_logger().error(
            f'MISSION ABORT: {reason}'
        )

        self.change_state('ABORT')

        if (
            self.vehicle_status is not None
            and self.vehicle_status.arming_state
            == self.ARMING_STATE_ARMED
        ):

            self.land()

            self.last_land_request = (
                self.now_seconds()
            )

    def change_state(self, new_state):

        self.state = new_state
        self.state_start_time = (
            self.get_clock().now()
        )

        self.get_logger().info(
            f'STATE -> {new_state}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = GPSDeniedWaypoint()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
