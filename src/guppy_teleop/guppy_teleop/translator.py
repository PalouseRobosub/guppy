import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray


class Translator(Node):
    def __init__(self):
        super().__init__("translator")

        self.dpad_subscriber = self.create_subscription(
            Int32MultiArray, "dpad", self.dpad_callback, 10
        )
        self.axes_subscriber = self.create_subscription(
            Float32MultiArray, "axes", self.axes_callback, 10
        )
        self.button_subscriber = self.create_subscription(
            Int32MultiArray, "buttons", self.button_callback, 10
        )

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.controller_state = {"dpad": None, "axes": None, "buttons": None}

    def dpad_callback(self, msg):
        self.get_logger().info("Received dpad message: %s" % msg.data)
        self.controller_state["dpad"] = msg.data
        self.update_controller_state()

    def axes_callback(self, msg):
        self.get_logger().info("Received axes message: %s" % msg.data)
        self.controller_state["axes"] = msg.data
        self.update_controller_state()

    def button_callback(self, msg):
        self.get_logger().info("Received button message: %s" % msg.data)
        self.controller_state["buttons"] = msg.data
        self.update_controller_state()

    def update_controller_state(self):
        if (
            self.controller_state["dpad"] is not None
            and self.controller_state["axes"] is not None
            and self.controller_state["buttons"] is not None
        ):
            self.publish_twist()
            self.controller_state = {"dpad": None, "axes": None, "buttons": None}

    def publish_twist(self):
        if (
            self.controller_state["dpad"] is not None
            and self.controller_state["axes"] is not None
            and self.controller_state["buttons"] is not None
        ):
            twist = Twist()
            twist.linear.x = self.controller_state["axes"][3] # right stick
            twist.linear.y = -self.controller_state["axes"][4] # right stick
            twist.linear.z = -self.controller_state["axes"][1] # left stick
            twist.angular.x = float(self.controller_state["dpad"][1]) # pitch
            twist.angular.y = float(self.controller_state["dpad"][0]) # roll
            yaw_r = self.controller_state["axes"][5] # right trigger
            yaw_l = self.controller_state["axes"][2] # left trigger
            yaw_r = (yaw_r + 1) / 2
            yaw_l = (yaw_l + 1) / 2 * (-1)
            twist.angular.z = yaw_r + yaw_l
            self.publisher.publish(twist)


def main(Args=None):
    rclpy.init(args=Args)

    translator = Translator()

    rclpy.spin(translator)

    translator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
