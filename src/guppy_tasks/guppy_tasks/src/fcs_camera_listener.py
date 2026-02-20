#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class FCSCameraListener(Node):
    def __init__(self):
        super().__init__('fcs_camera_listener')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/guppy/FCS_camera',
            self.image_callback,
            10
        )
        self.get_logger().info('FCS camera listener started, waiting for images...')

    def image_callback(self, msg: Image):
        self.get_logger().info(
            f'Received image: {msg.width}x{msg.height}, encoding: {msg.encoding}'
        )
        # Convert ROS image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Display the image
        cv2.imshow('FCS Camera', frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = FCSCameraListener()
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
