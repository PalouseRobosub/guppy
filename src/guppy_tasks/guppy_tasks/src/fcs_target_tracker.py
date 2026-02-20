#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class FCSTargetTracker(Node):
    def __init__(self):
        super().__init__('fcs_target_tracker')

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            '/guppy/FCS_camera',
            self.image_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Proportional gain for yaw correction
        self.kp_yaw = 0.005

        # Image center (updated on first frame)
        self.image_width = 640
        self.image_height = 480

        self.get_logger().info('FCS target tracker started.')

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.image_width = frame.shape[1]
        self.image_height = frame.shape[0]
        image_cx = self.image_width / 2.0

        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Red wraps around in HSV — need two ranges
        lower_red1 = np.array([0,   120,  70])
        upper_red1 = np.array([10,  255, 255])
        lower_red2 = np.array([170, 120,  70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Remove noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()

        if contours:
            # Use the largest contour
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > 100:  # Minimum area threshold
                M = cv2.moments(largest)
                if M['m00'] > 0:
                    cx = M['m10'] / M['m00']
                    cy = M['m01'] / M['m00']

                    # Horizontal error: positive = target is right of center
                    error_x = cx - image_cx

                    # Yaw to point at target (negative = turn right when target is right)
                    twist.angular.z = -self.kp_yaw * error_x

                    self.get_logger().info(
                        f'Target detected at ({cx:.0f}, {cy:.0f}), '
                        f'error_x={error_x:.1f}, yaw_cmd={twist.angular.z:.3f}'
                    )

                    # Draw on frame for debug
                    cv2.circle(frame, (int(cx), int(cy)), 10, (0, 255, 0), -1)
                    cv2.drawContours(frame, [largest], -1, (0, 255, 0), 2)
            else:
                self.get_logger().debug('Contour too small, ignoring.')
        else:
            self.get_logger().debug('No red target detected.')

        self.cmd_vel_pub.publish(twist)

        # Show debug windows
        cv2.imshow('FCS Camera', frame)
        cv2.imshow('Red Mask', mask)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = FCSTargetTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()