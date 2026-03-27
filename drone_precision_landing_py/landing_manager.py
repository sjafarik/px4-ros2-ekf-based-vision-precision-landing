#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, TwistStamped
from std_msgs.msg import Bool, Float32


class LandingManager(Node):
    def __init__(self):
        super().__init__('landing_manager')

        self.declare_parameter('k_xy', 0.8)
        self.declare_parameter('max_step_xy', 0.25)
        self.declare_parameter('alignment_threshold', 0.12)
        self.declare_parameter('alignment_hold_required', 5)
        self.declare_parameter('descent_step', 0.08)
        self.declare_parameter('landing_trigger_z', -0.8)
        self.declare_parameter('lost_pad_limit', 20)
        self.declare_parameter('landing_yaw', 0.0)

        self.k_xy = float(self.get_parameter('k_xy').value)
        self.max_step_xy = float(self.get_parameter('max_step_xy').value)
        self.alignment_threshold = float(self.get_parameter('alignment_threshold').value)
        self.alignment_hold_required = int(self.get_parameter('alignment_hold_required').value)
        self.descent_step = float(self.get_parameter('descent_step').value)
        self.landing_trigger_z = float(self.get_parameter('landing_trigger_z').value)
        self.lost_pad_limit = int(self.get_parameter('lost_pad_limit').value)
        self.landing_yaw = float(self.get_parameter('landing_yaw').value)

        self.landing_mode = False
        self.pad_visible = False
        self.have_position = False
        self.land_sent = False

        self.current_position = Point()
        self.target = Point()

        self.ex = 0.0
        self.ey = 0.0

        self.alignment_hold_counter = 0
        self.pad_lost_counter = 0
        self.last_log_time = 0.0

        self.create_subscription(Bool, '/mission/landing_mode', self.landing_mode_cb, 10)
        self.create_subscription(Bool, '/landing_pad/visible', self.visible_cb, 10)
        self.create_subscription(TwistStamped, '/landing_pad/state_estimate', self.ekf_cb, 10)
        self.create_subscription(Point, '/mission/current_position', self.position_cb, 10)

        self.target_pub = self.create_publisher(Point, '/mission/target_position', 10)
        self.target_yaw_pub = self.create_publisher(Float32, '/mission/target_yaw', 10)
        self.land_pub = self.create_publisher(Bool, '/mission/land', 10)

        self.timer = self.create_timer(0.1, self.timer_cb)

        self.get_logger().info('Landing manager started')

    def landing_mode_cb(self, msg: Bool):
        was_enabled = self.landing_mode
        self.landing_mode = msg.data

        if self.landing_mode and not was_enabled and self.have_position:
            self.target.x = self.current_position.x
            self.target.y = self.current_position.y
            self.target.z = self.current_position.z
            self.alignment_hold_counter = 0
            self.pad_lost_counter = 0
            self.get_logger().info(
                f'Landing handoff at ({self.target.x:.2f}, {self.target.y:.2f}, {self.target.z:.2f})'
            )

    def visible_cb(self, msg: Bool):
        self.pad_visible = msg.data
        if self.pad_visible:
            self.pad_lost_counter = 0

    def ekf_cb(self, msg: TwistStamped):
        self.ex = float(msg.twist.linear.x)
        self.ey = float(msg.twist.linear.y)

    def position_cb(self, msg: Point):
        self.current_position = msg
        self.have_position = True

    def maybe_log(self, text: str, period_sec: float = 1.0):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_log_time >= period_sec:
            self.get_logger().info(text)
            self.last_log_time = now

    def clamp(self, value: float, limit: float) -> float:
        return max(min(value, limit), -limit)

    def timer_cb(self):
        if not self.landing_mode or self.land_sent or not self.have_position:
            return

        yaw_msg = Float32()
        yaw_msg.data = self.landing_yaw
        self.target_yaw_pub.publish(yaw_msg)

        if not self.pad_visible:
            self.pad_lost_counter += 1
            self.maybe_log(f'Pad not visible. hold_cycles={self.pad_lost_counter}')
            if self.pad_lost_counter <= self.lost_pad_limit:
                self.target_pub.publish(self.target)
            return

        # IMPORTANT:
        # From your log, ex is the dominant error and it was driving Y the wrong way.
        # Fix: keep swapped mapping, but flip the Y sign.
        dx = self.clamp(-self.k_xy * self.ey, self.max_step_xy)
        dy = self.clamp(+self.k_xy * self.ex, self.max_step_xy)

        self.target.x = self.current_position.x + dx
        self.target.y = self.current_position.y + dy

        aligned = (
            abs(self.ex) < self.alignment_threshold and
            abs(self.ey) < self.alignment_threshold
        )

        if aligned:
            self.alignment_hold_counter += 1
        else:
            self.alignment_hold_counter = 0

        # In your setup, landing means z should move toward 0.0
        if self.alignment_hold_counter >= self.alignment_hold_required:
            self.target.z = min(self.current_position.z + self.descent_step, 0.0)
        else:
            self.target.z = self.current_position.z

        self.maybe_log(
            f'Landing | pos=({self.current_position.x:.2f}, {self.current_position.y:.2f}, {self.current_position.z:.2f}) '
            f'| err=({self.ex:.3f}, {self.ey:.3f}) '
            f'| dxy=({dx:.2f}, {dy:.2f}) '
            f'| aligned={aligned} hold={self.alignment_hold_counter} '
            f'| target=({self.target.x:.2f}, {self.target.y:.2f}, {self.target.z:.2f})'
        )

        if self.current_position.z >= self.landing_trigger_z:
            self.get_logger().info('Final landing triggered')
            land_msg = Bool()
            land_msg.data = True
            self.land_pub.publish(land_msg)
            self.land_sent = True
            return

        self.target_pub.publish(self.target)


def main(args=None):
    rclpy.init(args=args)
    node = LandingManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()