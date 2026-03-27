#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float32

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus


class OffboardControlNode(Node):
    def __init__(self) -> None:
        super().__init__('offboard_control_node')

        # -------------------------------------------------
        # Parameters
        # -------------------------------------------------
        self.declare_parameter('default_target_x', 0.0)
        self.declare_parameter('default_target_y', 0.0)
        self.declare_parameter('default_target_z', -5.0)
        self.declare_parameter('default_target_yaw', 0.0)
        self.declare_parameter('setpoint_rate_hz', 10.0)

        self.default_target_x = float(self.get_parameter('default_target_x').value)
        self.default_target_y = float(self.get_parameter('default_target_y').value)
        self.default_target_z = float(self.get_parameter('default_target_z').value)
        self.default_target_yaw = float(self.get_parameter('default_target_yaw').value)
        self.setpoint_rate_hz = float(self.get_parameter('setpoint_rate_hz').value)

        if self.setpoint_rate_hz <= 0.0:
            raise ValueError('Parameter "setpoint_rate_hz" must be > 0.0')

        self.timer_period = 1.0 / self.setpoint_rate_hz

        # -------------------------------------------------
        # QoS
        # -------------------------------------------------
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        ros_qos = 10

        # -------------------------------------------------
        # PX4 publishers
        # -------------------------------------------------
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            px4_qos
        )

        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            px4_qos
        )

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            px4_qos
        )

        # -------------------------------------------------
        # PX4 subscribers
        # -------------------------------------------------
        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            px4_qos
        )

        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            px4_qos
        )

        # -------------------------------------------------
        # Mission / landing interface
        # -------------------------------------------------
        self.target_position_sub = self.create_subscription(
            Point,
            '/mission/target_position',
            self.target_position_callback,
            ros_qos
        )

        self.target_yaw_sub = self.create_subscription(
            Float32,
            '/mission/target_yaw',
            self.target_yaw_callback,
            ros_qos
        )

        self.land_request_sub = self.create_subscription(
            Bool,
            '/mission/land',
            self.land_request_callback,
            ros_qos
        )

        self.current_position_pub = self.create_publisher(
            Point,
            '/mission/current_position',
            ros_qos
        )

        # -------------------------------------------------
        # Internal state
        # -------------------------------------------------
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        self.have_local_position = False
        self.land_requested = False
        self.land_command_sent = False

        self.target_x = self.default_target_x
        self.target_y = self.default_target_y
        self.target_z = self.default_target_z
        self.target_yaw = self.default_target_yaw

        self.last_logged_target = None
        self.offboard_setpoint_counter = 0

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # -------------------------------------------------
        # Startup logs
        # -------------------------------------------------
        self.get_logger().info('Offboard control node started')
        self.get_logger().info(
            f'Default target: ({self.target_x:.2f}, {self.target_y:.2f}, {self.target_z:.2f}), '
            f'yaw={self.target_yaw:.2f}, setpoint_rate_hz={self.setpoint_rate_hz:.2f}'
        )

    # ------------------------------------------------------------------
    # PX4 callbacks
    # ------------------------------------------------------------------
    def vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        self.vehicle_local_position = msg
        self.have_local_position = True

        current_position_msg = Point()
        current_position_msg.x = float(msg.x)
        current_position_msg.y = float(msg.y)
        current_position_msg.z = float(msg.z)
        self.current_position_pub.publish(current_position_msg)

    def vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg

    # ------------------------------------------------------------------
    # Mission callbacks
    # ------------------------------------------------------------------
    def target_position_callback(self, msg: Point) -> None:
        self.target_x = float(msg.x)
        self.target_y = float(msg.y)
        self.target_z = float(msg.z)

        rounded_target = (
            round(self.target_x, 2),
            round(self.target_y, 2),
            round(self.target_z, 2),
            round(self.target_yaw, 2),
        )

        if rounded_target != self.last_logged_target:
            self.get_logger().info(
                f'Updated target position: '
                f'x={self.target_x:.2f}, y={self.target_y:.2f}, z={self.target_z:.2f}, yaw={self.target_yaw:.2f}'
            )
            self.last_logged_target = rounded_target

    def target_yaw_callback(self, msg: Float32) -> None:
        self.target_yaw = float(msg.data)

    def land_request_callback(self, msg: Bool) -> None:
        if msg.data and not self.land_requested and not self.land_command_sent:
            self.land_requested = True
            self.get_logger().info('Landing requested by mission system')

    # ------------------------------------------------------------------
    # Main timer
    # ------------------------------------------------------------------
    def timer_callback(self) -> None:
        self.publish_offboard_control_mode()

        if not self.land_command_sent:
            self.publish_trajectory_setpoint()

        if self.offboard_setpoint_counter == 10:
            self.get_logger().info('Requesting OFFBOARD mode...')
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                param1=1.0,
                param2=6.0
            )

            self.get_logger().info('Requesting ARM...')
            self.arm()

        if self.land_requested and self.offboard_setpoint_counter >= 11:
            self.get_logger().info('Sending LAND command to PX4...')
            self.land()
            self.land_requested = False
            self.land_command_sent = True

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

    # ------------------------------------------------------------------
    # PX4 publishers
    # ------------------------------------------------------------------
    def publish_offboard_control_mode(self) -> None:
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.get_timestamp_us()
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self) -> None:
        msg = TrajectorySetpoint()
        msg.position = [self.target_x, self.target_y, self.target_z]
        msg.yaw = self.target_yaw
        msg.timestamp = self.get_timestamp_us()
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(
        self,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0
    ) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_timestamp_us()
        self.vehicle_command_pub.publish(msg)

    # ------------------------------------------------------------------
    # Vehicle commands
    # ------------------------------------------------------------------
    def arm(self) -> None:
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0
        )

    def disarm(self) -> None:
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0
        )

    def land(self) -> None:
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------
    def get_timestamp_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OffboardControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down offboard node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()