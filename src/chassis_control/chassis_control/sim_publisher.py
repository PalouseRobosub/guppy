import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray, Float32

class SimPublisher(Node):
    pubs = []

    def __init__(self):
        super().__init__('sim_publisher')

        self.sub = self.create_subscription(Float32MultiArray, '/chassis_control/motor_outputs', self.callback, 10)

        for i in range(8):
            self.pubs.append(self.create_publisher(Float32, '/motor/m'+str(i), 10))

    def callback(self, msg):
        self.get_logger().info('I heard: "%s"' % str(msg))

        for pub, data in zip(self.pubs, msg.data):
            out = Float32()
            out.data = data
            pub.publish(out)

def main(args=None):
    rclpy.init(args=args)

    node = SimPublisher()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
