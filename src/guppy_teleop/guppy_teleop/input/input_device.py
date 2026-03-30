from time import time

from abc import ABC, abstractmethod

from geometry_msgs.msg import Twist

from guppy_teleop.util.device_priority import DevicePriority

# TODO replace enabled with a mode: DISABLED, COMMAND, INPUT so that a device can only send commands (ie. the keyboard so it doesn't send things while typing)
class InputDevice(ABC):
    def __init__(self, enabled: bool = False, name: str = "Unkown Input Device", priority: DevicePriority = DevicePriority.MEDIUM):
        self.handler = None # bad design but whatever, makes it convienient to call commands without passing in lots of callbacks
        
        self._enabled = enabled
        self.name = name
        self.priority = priority

        self.active = False
        self.last_active = 0.0

        self._command_mode = False

        self._state: dict = None

    def is_enabled(self) -> bool:
        return self._enabled

    def enable(self):
        self._enabled = True

    def disable(self):
        self._enabled = False
        self.active = False

    def _mark_active(self):
        self.active = True
        self.last_active = time()

    def _mark_inactive(self):
        self.active = False

    @abstractmethod
    async def start(self):
        ...

    @abstractmethod
    def transform(self, snapshot: dict) -> Twist:
        ...

    @abstractmethod
    def package(self, snapshot: dict) -> dict:
        ...
