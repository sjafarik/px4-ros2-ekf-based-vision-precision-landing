#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool

from cv_bridge import CvBridge
import cv2
import numpy as np


class PadDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('pad_detector_node')

        # -------------------------------------------------
        # Parameters
        # -------------------------------------------------
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('min_area', 500.0)
        self.declare_parameter('debug_view', True)

        # HSV threshold for red (range 1)
        self.declare_parameter('red1_h_min', 0)
        self.declare_parameter('red1_h_max', 10)
        self.declare_parameter('red1_s_min', 100)
        self.declare_parameter('red1_s_max', 255)
        self.declare_parameter('red1_v_min', 100)
        self.declare_parameter('red1_v_max', 255)

        # HSV threshold for red (range 2)
        self.declare_parameter('red2_h_min', 160)
        self.declare_parameter('red2_h_max', 179)
        self.declare_parameter('red2_s_min', 100)
        self.declare_parameter('red2_s_max', 255)
        self.declare_parameter('red2_v_min', 100)
        self.declare_parameter('red2_v_max', 255)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.min_area = float(self.get_parameter('min_area').value)
        self.debug_view = bool(self.get_parameter('debug_view').value)

        self.red1_lower = np.array([
            int(self.get_parameter('red1_h_min').value),
            int(self.get_parameter('red1_s_min').value),
            int(self.get_parameter('red1_v_min').value)
        ], dtype=np.uint8)

        self.red1_upper = np.array([
            int(self.get_parameter('red1_h_max').value),
            int(self.get_parameter('red1_s_max').value),
            int(self.get_parameter('red1_v_max').value)
        ], dtype=np.uint8)

        self.red2_lower = np.array([
            int(self.get_parameter('red2_h_min').value),
            int(self.get_parameter('red2_s_min').value),
            int(self.get_parameter('red2_v_min').value)
        ], dtype=np.uint8)

        self.red2_upper = np.array([
            int(self.get_parameter('red2_h_max').value),
            int(self.get_parameter('red2_s_max').value),
            int(self.get_parameter('red2_v_max').value)
        ], dtype=np.uint8)

        # -------------------------------------------------
        # ROS interfaces
        # -------------------------------------------------
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.measurement_pub = self.create_publisher(
            PointStamped,
            '/landing_pad/measurement',
            10
        )

        self.visible_pub = self.create_publisher(
            Bool,
            '/landing_pad/visible',
            10
        )

        self.get_logger().info('Pad detector node started')
        self.get_logger().info(f'Subscribed image topic: {self.image_topic}')
        self.get_logger().info(f'Min area threshold: {self.min_area:.1f}')
        self.get_logger().info(f'Debug view: {self.debug_view}')

    # ------------------------------------------------------------------
    # Main image callback
    # ------------------------------------------------------------------
    def image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Failed to convert image: {exc}')
            return

        if frame is None or frame.size == 0:
            return

        frame_height, frame_width = frame.shape[:2]
        image_center_x = frame_width / 2.0
        image_center_y = frame_height / 2.0

        # Convert to HSV for easier color thresholding
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Red wraps around in HSV, so combine two masks
        mask1 = cv2.inRange(hsv, self.red1_lower, self.red1_upper)
        mask2 = cv2.inRange(hsv, self.red2_lower, self.red2_upper)
        mask = cv2.bitwise_or(mask1, mask2)

        # Clean up noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        visible_msg = Bool()
        visible_msg.data = False

        best_contour = None
        best_area = 0.0

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > best_area:
                best_area = area
                best_contour = contour

        if best_contour is not None and best_area >= self.min_area:
            moments = cv2.moments(best_contour)

            if abs(moments['m00']) > 1e-6:
                pad_center_x = moments['m10'] / moments['m00']
                pad_center_y = moments['m01'] / moments['m00']

                # Normalized image error:
                # x < 0 means pad is left of image center
                # x > 0 means pad is right of image center
                # y < 0 means pad is above image center
                # y > 0 means pad is below image center
                error_x = (pad_center_x - image_center_x) / image_center_x
                error_y = (pad_center_y - image_center_y) / image_center_y

                normalized_area = best_area / float(frame_width * frame_height)

                measurement_msg = PointStamped()
                measurement_msg.header = msg.header
                measurement_msg.point.x = float(error_x)
                measurement_msg.point.y = float(error_y)
                measurement_msg.point.z = float(normalized_area)

                self.measurement_pub.publish(measurement_msg)

                visible_msg.data = True

                if self.debug_view:
                    debug_frame = frame.copy()

                    cv2.drawContours(debug_frame, [best_contour], -1, (0, 255, 0), 2)
                    cv2.circle(
                        debug_frame,
                        (int(pad_center_x), int(pad_center_y)),
                        6,
                        (255, 0, 0),
                        -1
                    )
                    cv2.circle(
                        debug_frame,
                        (int(image_center_x), int(image_center_y)),
                        6,
                        (0, 255, 255),
                        -1
                    )

                    cv2.putText(
                        debug_frame,
                        f"err_x={error_x:.3f}, err_y={error_y:.3f}",
                        (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (255, 255, 255),
                        2
                    )

                    cv2.putText(
                        debug_frame,
                        f"area={best_area:.0f}, norm_area={normalized_area:.4f}",
                        (20, 60),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (255, 255, 255),
                        2
                    )

                    cv2.imshow('pad_detector_debug', debug_frame)
                    cv2.waitKey(1)

        self.visible_pub.publish(visible_msg)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PadDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down pad detector node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()