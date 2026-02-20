#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np


# Known real-world diameter of the hoop in meters
KNOWN_DIAMETER_M = 0.2

# Camera horizontal FOV in radians (60 degrees = 1.047 rad, matches URDF)
HFOV_RAD = 1.047


class FCSRangefinder(Node):
    def __init__(self):
        super().__init__('fcs_rangefinder')

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            '/guppy/FCS_camera',
            self.image_callback,
            10
        )

        # Publishes range in meters
        self.range_pub = self.create_publisher(Float32, '/fcs/target_range', 10)

        self.image_width = 640
        self.image_height = 480

        self.get_logger().info('FCS rangefinder started.')

    def get_focal_length_px(self):
        """
        Derive focal length in pixels from image width and HFOV.
        f = (image_width / 2) / tan(HFOV / 2)
        """
        return (self.image_width / 2.0) / np.tan(HFOV_RAD / 2.0)

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.image_width = frame.shape[1]
        self.image_height = frame.shape[0]

        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Red HSV ranges
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

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > 100:
                # Fit a circle to the contour to get pixel diameter
                (cx, cy), radius_px = cv2.minEnclosingCircle(largest)
                diameter_px = radius_px * 2.0

                if diameter_px > 0:
                    # Range from similar triangles:
                    # range = (known_diameter * focal_length) / diameter_px
                    focal_length_px = self.get_focal_length_px()
                    range_m = (KNOWN_DIAMETER_M * focal_length_px) / diameter_px

                    self.get_logger().info(
                        f'Diameter: {diameter_px:.1f}px | Range: {range_m:.2f}m'
                    )

                    range_msg = Float32()
                    range_msg.data = float(range_m)
                    self.range_pub.publish(range_msg)

                    # Debug overlay
                    cv2.circle(frame, (int(cx), int(cy)), int(radius_px), (0, 255, 0), 2)
                    cv2.putText(
                        frame,
                        f'{range_m:.2f}m',
                        (int(cx) - 40, int(cy) - int(radius_px) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2
                    )
        else:
            self.get_logger().debug('No red target detected.')

        cv2.imshow('FCS Rangefinder', frame)
        cv2.imshow('Red Mask', mask)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = FCSRangefinder()
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