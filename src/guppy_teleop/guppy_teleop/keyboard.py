from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
import rclpy

class KeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__("keyboard_teleop")

        pygame.init()
        pygame.display.set_mode((100, 100))

        self.publisher = self.create_publisher(Twist, "/cmd_vel/teleop", 10)

        self.linear_speed = 1
        self.angular_speed = 0.5

        self.timer = self.create_timer(0.05, self.get_input)

    def get_input(self) -> None:
        pygame.event.pump()

        pressed = pygame.key.get_pressed()
        twist = Twist()

        if pressed[pygame.K_w]:
            twist.linear.x += self.linear_speed
        if pressed[pygame.K_s]:
            twist.linear.x -= self.linear_speed

        if pressed[pygame.K_d]:
            twist.linear.y += self.linear_speed
        if pressed[pygame.K_a]:
            twist.linear.y -= self.linear_speed

        if pressed[pygame.K_e]:
            twist.linear.z += self.linear_speed
        if pressed[pygame.K_q]:
            twist.linear.z -= self.linear_speed

        if pressed[pygame.K_UP]:
            twist.angular.x += self.angular_speed
        if pressed[pygame.K_DOWN]:
            twist.angular.x -= self.angular_speed

        if pressed[pygame.K_RIGHT]:
            twist.angular.y += self.angular_speed
        if pressed[pygame.K_LEFT]:
            twist.angular.y -= self.angular_speed

        if pressed[pygame.K_p]:
            twist.angular.z += self.angular_speed
        if pressed[pygame.K_o]:
            twist.angular.z -= self.angular_speed

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()