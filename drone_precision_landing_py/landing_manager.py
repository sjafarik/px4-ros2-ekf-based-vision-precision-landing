#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, TwistStamped
from std_msgs.msg import Bool


class LandingManager(Node):
    def __init__(self):
        super().__init__('landing_manager')

        # -------------------------------------------------
        # Parameters
        # -------------------------------------------------
        self.declare_parameter('k_xy', 1.5)
        self.declare_parameter('max_step_xy', 0.5)
        self.declare_parameter('descent_rate', 0.02)
        self.declare_parameter('alignment_threshold', 0.05)
        self.declare_parameter('min_altitude', -0.5)

        self.k_xy = float(self.get_parameter('k_xy').value)
        self.max_step_xy = float(self.get_parameter('max_step_xy').value)
        self.descent_rate = float(self.get_parameter('descent_rate').value)
        self.alignment_threshold = float(self.get_parameter('alignment_threshold').value)
        self.min_altitude = float(self.get_parameter('min_altitude').value)

        # -------------------------------------------------
        # State
        # -------------------------------------------------
        self.landing_mode = False
        self.pad_visible = False

        self.current_position = Point()
        self.current_target = Point()

        self.ex = 0.0
        self.ey = 0.0

        self.land_sent = False

        # -------------------------------------------------
        # Subscribers
        # -------------------------------------------------
        self.create_subscription(Bool, '/mission/landing_mode', self.landing_mode_cb, 10)
        self.create_subscription(Bool, '/landing_pad/visible', self.visible_cb, 10)
        self.create_subscription(TwistStamped, '/landing_pad/state_estimate', self.ekf_cb, 10)
        self.create_subscription(Point, '/mission/current_position', self.position_cb, 10)

        # -------------------------------------------------
        # Publishers
        # -------------------------------------------------
        self.target_pub = self.create_publisher(Point, '/mission/target_position', 10)
        self.land_pub = self.create_publisher(Bool, '/mission/land', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.timer_cb)

        self.get_logger().info('Landing manager started')

    # -------------------------------------------------
    # Callbacks
    # -------------------------------------------------
    def landing_mode_cb(self, msg):
        self.landing_mode = msg.data

    def visible_cb(self, msg):
        self.pad_visible = msg.data

    def ekf_cb(self, msg):
        self.ex = msg.twist.linear.x
        self.ey = msg.twist.linear.y

    def position_cb(self, msg):
        self.current_position = msg
        self.current_target = msg

    # -------------------------------------------------
    # Main logic
    # -------------------------------------------------
    def timer_cb(self):
        if not self.landing_mode:
            return

        if self.land_sent:
            return

        if not self.pad_visible:
            self.get_logger().warn_throttle(2.0, 'Pad not visible')
            return

        target = Point()
        target.x = self.current_position.x
        target.y = self.current_position.y
        target.z = self.current_position.z

        # -------------------------------------------------
        # Convert image error to motion
        # -------------------------------------------------
        dx = -self.k_xy * self.ex
        dy = -self.k_xy * self.ey

        # Clamp movement
        dx = max(min(dx, self.max_step_xy), -self.max_step_xy)
        dy = max(min(dy, self.max_step_xy), -self.max_step_xy)

        target.x += dx
        target.y += dy

        # -------------------------------------------------
        # Check alignment
        # -------------------------------------------------
        aligned = abs(self.ex) < self.alignment_threshold and abs(self.ey) < self.alignment_threshold

        if aligned:
            target.z += self.descent_rate  # remember: NED (negative down)

        # -------------------------------------------------
        # Check landing condition
        # -------------------------------------------------
        if self.current_position.z > self.min_altitude:
            self.get_logger().info('Final landing triggered')
            self.send_land()
            return

        self.target_pub.publish(target)

    # -------------------------------------------------
    def send_land(self):
        msg = Bool()
        msg.data = True
        self.land_pub.publish(msg)
        self.land_sent = True


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