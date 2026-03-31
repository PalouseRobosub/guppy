from time import time

from abc import ABC, abstractmethod

from geometry_msgs.msg import Twist

from guppy_teleop.util.device_priority import DevicePriority
from guppy_teleop.util.device_mode import DeviceMode

# TODO replace enabled with a mode: DISABLED, COMMAND, INPUT so that a device can only send commands (ie. the keyboard so it doesn't send things while typing)
class InputDevice(ABC):
    def __init__(self, mode: DeviceMode = DeviceMode.DISABLED, name: str = "Unkown Input Device", priority: DevicePriority = DevicePriority.MEDIUM):
        self.handler = None # bad design but whatever, makes it convienient to call commands without passing in lots of callbacks
        
        self.mode = mode
        self.name = name
        self.priority = priority

        self.active = False
        self.last_active = 0.0

        self._command_mode = False

        self._state: dict = None

    def _mark_active(self):
        self.active = True
        self.last_active = time()

    def _mark_inactive(self):
        self.active = False
    
    def _cycle_mode(self): # TODO devices need better control rather than just cycling, or the method of cycling is still available while disabled
        self.mode = self.mode.next()

    @abstractmethod
    async def start(self):
        ...

    @abstractmethod
    def transform(self, snapshot: dict) -> Twist:
        ...

    @abstractmethod
    def package(self, snapshot: dict) -> dict:
        ...
