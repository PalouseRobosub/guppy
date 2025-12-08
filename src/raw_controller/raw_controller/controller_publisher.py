#!/usr/bin/env python3

from typing import Any, Dict

import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray


class RawController:
    def __init__(self) -> None:
        pygame.init()
        pygame.joystick.init()
        self._joystick = None

        # search for controllers
        for i in range(pygame.joystick.get_count()):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()

            if joystick:
                self._joystick = joystick
                break

        if self._joystick is None:
            raise ValueError("No controller found")

        # get controller layout
        self.numaxes = self._joystick.get_numaxes()
        self.numbuttons = self._joystick.get_numbuttons()
        self.numhats = self._joystick.get_numhats()

    def update(self) -> Dict[str, Any]:
        if self._joystick is None:
            raise ValueError("No controller found")
        pygame.event.pump()
        state = {}
        state["axes"] = [self._joystick.get_axis(i) for i in range(self.numaxes)]
        state["buttons"] = [
            self._joystick.get_button(i) for i in range(self.numbuttons)
        ]
        state["hats"] = [self._joystick.get_hat(i) for i in range(self.numhats)]
        return state


class RawControllerPublisher(Node):
    def __init__(self):
        super().__init__("controller_publisher")
        # create publishers
        self.dpad_publisher = self.create_publisher(Int32MultiArray, "dpad", 10)
        self.axes_publisher = self.create_publisher(Float32MultiArray, "axes", 10)
        self.button_publisher = self.create_publisher(Int32MultiArray, "buttons", 10)
        
        self.controller = RawController()
        self.timer = self.create_timer(0.05, self.publish_controller)  # 20 Hz

    def publish_controller(self):
        try:
            state = self.controller.update()
            dpad_msg = Int32MultiArray()
            dpad_msg.data = [item for pair in state["hats"] for item in pair]
            self.dpad_publisher.publish(dpad_msg)
            axes_msg = Float32MultiArray()
            axes_msg.data = state["axes"]
            self.axes_publisher.publish(axes_msg)
            button_msg = Int32MultiArray()
            button_msg.data = state["buttons"]
            self.button_publisher.publish(button_msg)
            print(f"Published dpad: {dpad_msg.data}")
            print(f"Published axes: {axes_msg.data}")
            print(f"Published buttons: {button_msg.data}")
        except Exception as e:
            self.get_logger().error(f"Controller read failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = RawControllerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
