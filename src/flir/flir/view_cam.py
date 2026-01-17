import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageViewer(Node):
    def __init__(self):
        super().__init__('image_viewer')
        self.bridge = CvBridge()

        # Subscriber to the /flir_camera/image_raw topic
        self.subscription = self.create_subscription(
            Image,
            '/flir_camera/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info('Image Viewer node has started.')
        cv2.namedWindow('Live Camera Feed', cv2.WINDOW_NORMAL)

    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to a OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Display the image in a window
            cv2.imshow('Live Camera Feed', cv_image)
            cv2.waitKey(1)  # Allow OpenCV to update the window
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

def main(args=None):
    rclpy.init(args=args)

    node = ImageViewer()

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
