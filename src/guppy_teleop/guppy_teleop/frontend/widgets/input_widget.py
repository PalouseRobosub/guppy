from rclpy.node import Node

import random

from evdev import KeyEvent

from PySide6.QtCore import Property, Signal

from guppy_teleop.frontend.widgets.widget import Widget

from guppy_teleop.input_handler import InputHandler
from guppy_teleop.input.controller import find_controllers
from guppy_teleop.input.keyboard import find_keyboards

class InputWidget(InputHandler, Widget):
    inputUpdated = Signal()

    @property
    def qml_name(self) -> str:
        return "inputWidget"    

    def __init__(self, parent=None):
        InputHandler.__init__(self, self.update)
        Widget.__init__(self, parent)

        self.add_device(*find_controllers(True))
        self.add_device(*find_keyboards(True))

        self._input_package = {
            "name": "test keyboard",
            "format": "keyboard",
            "input": {
                "q": KeyEvent.key_up,
                "w": KeyEvent.key_up,
                "e": KeyEvent.key_up,
                "a": KeyEvent.key_up,
                "s": KeyEvent.key_up,
                "d": KeyEvent.key_up,
                "o": KeyEvent.key_up,
                "p": KeyEvent.key_up,
                "up": KeyEvent.key_up,
                "down": KeyEvent.key_up,
                "left": KeyEvent.key_up,
                "right": KeyEvent.key_up
            }
        }

        self.thread_start()

        self.input_test_timer = self.create_timer(0.25, self._input_test)

    @Property("QVariantMap", notify=inputUpdated)
    def inputPackage(self) -> dict:
        return self._input_package
    
    def update(self, input_package: dict):
        self._input_package = input_package
        
        self.inputUpdated.emit()

    def _input_test(self):
        key, value = random.choice(list(self._input_package["input"].items()))
        self._input_package["input"][key] = KeyEvent.key_down if value == KeyEvent.key_up else KeyEvent.key_up
        self.inputUpdated.emit()