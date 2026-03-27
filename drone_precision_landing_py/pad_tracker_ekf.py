#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, TwistStamped
import numpy as np


class PadTrackerEKF(Node):
    def __init__(self) -> None:
        super().__init__('pad_tracker_ekf')

        # -------------------------------------------------
        # Parameters
        # -------------------------------------------------
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('q_pos', 0.01)
        self.declare_parameter('q_vel', 0.1)
        self.declare_parameter('r_meas', 0.05)

        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.q_pos = float(self.get_parameter('q_pos').value)
        self.q_vel = float(self.get_parameter('q_vel').value)
        self.r_meas = float(self.get_parameter('r_meas').value)

        self.dt = 1.0 / self.rate_hz

        # -------------------------------------------------
        # State: [ex, ey, vex, vey]
        # -------------------------------------------------
        self.x = np.zeros((4, 1))

        # Covariance
        self.P = np.eye(4)

        # -------------------------------------------------
        # Subscribers / Publishers
        # -------------------------------------------------
        self.measurement_sub = self.create_subscription(
            PointStamped,
            '/landing_pad/measurement',
            self.measurement_callback,
            10
        )

        self.state_pub = self.create_publisher(
            TwistStamped,
            '/landing_pad/state_estimate',
            10
        )

        # Timer for prediction loop
        self.timer = self.create_timer(1.0 / self.rate_hz, self.timer_callback)

        self.get_logger().info('EKF node started')

    # -------------------------------------------------
    # Prediction step (runs continuously)
    # -------------------------------------------------
    def predict(self):
        dt = self.dt

        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0 ],
            [0, 0, 0, 1 ]
        ])

        Q = np.array([
            [self.q_pos, 0, 0, 0],
            [0, self.q_pos, 0, 0],
            [0, 0, self.q_vel, 0],
            [0, 0, 0, self.q_vel]
        ])

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    # -------------------------------------------------
    # Measurement update (runs when detection arrives)
    # -------------------------------------------------
    def measurement_callback(self, msg: PointStamped):
        z = np.array([
            [msg.point.x],
            [msg.point.y]
        ])

        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        R = np.array([
            [self.r_meas, 0],
            [0, self.r_meas]
        ])

        # Innovation
        y = z - (H @ self.x)

        # Innovation covariance
        S = H @ self.P @ H.T + R

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update
        self.x = self.x + K @ y

        I = np.eye(4)
        self.P = (I - K @ H) @ self.P

    # -------------------------------------------------
    # Timer loop
    # -------------------------------------------------
    def timer_callback(self):
        self.predict()
        self.publish_state()

    # -------------------------------------------------
    # Publish filtered estimate
    # -------------------------------------------------
    def publish_state(self):
        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.twist.linear.x = float(self.x[0, 0])  # filtered ex
        msg.twist.linear.y = float(self.x[1, 0])  # filtered ey

        msg.twist.angular.x = float(self.x[2, 0]) # vex
        msg.twist.angular.y = float(self.x[3, 0]) # vey

        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PadTrackerEKF()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()