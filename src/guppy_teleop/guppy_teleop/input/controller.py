from evdev import InputDevice as EvdevDevice, ecodes, list_devices#, categorize

from geometry_msgs.msg import Twist

from guppy_teleop.input.input_device import InputDevice
from guppy_teleop.util.device_priority import DevicePriority
from guppy_teleop.util.code_map import CodeMap

from typing import Callable#, TYPE_CHECKING
#if TYPE_CHECKING:
#    from guppy_teleop.input_handler import InputHandler

class Controller(InputDevice):
    COMMAND_KEYS = [ecodes.BTN_SELECT]

    BLACKLISTED_DEVICES = [] #["AT Translated Set 2 keyboard"]

    DEADZONE = 0.1 # TODO, MAKE THIS BETTER. RAMPING FUNCTION SO IT MAXES OUT AND MINS OUT at 0.2 and 0.8

    LINEAR_MULTIPLIER = [1.0, 1.0, 1.0]
    ANGULAR_MULTIPLIER = [1.0, 1.0, 1.0]

    VALID_TYPES = [ecodes.EV_ABS, ecodes.EV_KEY]

    # default values for a Logitech Gamepad F310
    CODE_MAP = {
        "left_stick_x": ecodes.ABS_X,   # type 03, from -32768 to 32768
        "left_stick_y": ecodes.ABS_Y,   # type 03, from -32768 to 32768
        "left_trigger": ecodes.ABS_Z,   # type 03, from 0 to 255
        "right_stick_x": ecodes.ABS_RX, # type 03, from -32768 to 32768
        "right_stick_y": ecodes.ABS_RY, # type 03, from -32768 to 32768
        "right_trigger": ecodes.ABS_RZ, # type 03, from 0 to 255
        "dpad_x": ecodes.ABS_HAT0X,        # type 03, from -1 to 1
        "dpad_y": ecodes.ABS_HAT0Y,        # type 03, from -1 to 1
        "a": ecodes.BTN_A,            # type 01, 0 or 1
        "b": ecodes.BTN_B,            # type 01, 0 or 1
        "x": ecodes.BTN_X,            # type 01, 0 or 1
        "y": ecodes.BTN_Y,            # type 01, 0 or 1
        "left_bumper": ecodes.BTN_TL,    # type 01, 0 or 1
        "right_bumper": ecodes.BTN_TR,   # type 01, 0 or 1
        "select": ecodes.BTN_SELECT,     # type 01, 0 or 1
        "start": ecodes.BTN_START,       # type 01, 0 or 1
        "mode": ecodes.BTN_MODE,         # type 01, 0 or 1
        "left_thumb": ecodes.BTN_THUMBL, # type 01, 0 or 1
        "right_thumb": ecodes.BTN_THUMBR # type 01, 0 or 1
    }

    @staticmethod
    def normalize_int16(num: int):
        return num / 32768
    
    @staticmethod
    def normalize_uint8(num: int):
        return num / 255

    NORMALIZE_MAP = {
        "left_stick_x": normalize_int16,
        "left_stick_y": normalize_int16,
        "left_trigger": normalize_uint8,
        "right_stick_x": normalize_int16,
        "right_stick_y": normalize_int16,
        "right_trigger": normalize_uint8
    }

    _internal_code_map = CodeMap(CODE_MAP)

    def __init__(self, handler, path: str, enabled = False, name = None, priority = DevicePriority.MEDIUM):
        self._device = EvdevDevice(path)

        super().__init__(enabled, name if name else self._device.name, priority)

        self.handler = handler

        self.COMMAND_MAP: dict[Callable, list[int]] = {
            lambda: self.handler.push_state("FAULT"): [ecodes.BTN_SELECT, ecodes.BTN_B],
            lambda: self.handler.push_state("TELEOP"): [ecodes.BTN_SELECT, ecodes.BTN_A],
            lambda: self.handler.push_state("DISABLED"): [ecodes.BTN_SELECT, ecodes.BTN_X],
            lambda: self.handler.lock_priority(self.priority): [ecodes.BTN_SELECT, ecodes.BTN_TL],
            lambda: self.handler.lock_priority(None): [ecodes.BTN_SELECT, ecodes.BTN_TR]
        }

        self._state = {
            "left_stick_x": 0.0,   # normalized -1 -> 1
            "left_stick_y": 0.0,   # normalized -1 -> 1
            "left_trigger": 0.0,   # normalized 0 -> 1
            "right_stick_x": 0.0,  # normalized -1 -> 1
            "right_stick_y": 0.0,  # normalized -1 -> 1
            "right_trigger": 0.0,  # normalized 0 -> 1
            "dpad_x": 0,           # from -1 -> 1
            "dpad_y": 0,           # from -1 -> 1
            "a": False,            # True or False
            "b": False,            # True or False
            "x": False,            # True or False
            "y": False,            # True or False
            "left_bumper": False,  # True or False
            "right_bumper": False, # True or False
            "select": False,       # True or False
            "start": False,        # True or False
            "mode": False,         # True or False
            "left_thumb": False,    # True or False
            "right_thumb": False,  # True or False
        }

        print(f"'{self.name}' initialized!")

    async def start(self):
        async for event in self._device.async_read_loop():
            self._process(event)

    def _process(self, event):
        if event.type in self.VALID_TYPES:
            if self._command_mode:
                if (event.value) == 0: # TODO use enum for key up?
                    return

                active_keys = self._device.active_keys()

                if any(key in active_keys for key in self.COMMAND_KEYS):
                    for command, keys in self.COMMAND_MAP.items():
                        if (event.code not in keys):
                            return
                        
                        if all(key in active_keys for key in keys):
                            command() # remember to use try catch in your commands or it will crash with no error!

                            return

                    return
                else:
                    self._command_mode = False

                    if (event.code in self.COMMAND_KEYS):
                        return
            if (code_name := self._internal_code_map[event.code]) is not None:
                if (event.code in self.COMMAND_KEYS):
                    self._command_mode = True

                    return

                if (normalize := self.NORMALIZE_MAP.get(code_name)) is not None:
                    self._state[code_name] = normalize(event.value)
                else:
                    self._state[code_name] = event.value
                
                self._mark_active()
                if (self.handler):
                    self.handler.on_device_event(self._state.copy())
            
            print(f"'{code_name}' -> {self._state[code_name]}")

            #print(categorize(event))
            #print(self._device.active_keys(verbose=True))

    def transform(self, snapshot: dict) -> Twist:
        twist = Twist()

        twist.linear.x = float(snapshot["right_stick_x"]) * self.LINEAR_MULTIPLIER[0]
        twist.linear.y = -float(snapshot["right_stick_y"]) * self.LINEAR_MULTIPLIER[1]
        twist.linear.z = float(snapshot["left_stick_y"]) * self.LINEAR_MULTIPLIER[2]

        twist.angular.x = float(snapshot["dpad_x"]) * self.ANGULAR_MULTIPLIER[1]
        twist.angular.y = -float(snapshot["dpad_y"]) * self.ANGULAR_MULTIPLIER[2]
        twist.angular.z = float(snapshot["right_trigger"]) - float(snapshot["left_trigger"]) * self.ANGULAR_MULTIPLIER[3]

        return twist

    def package(self, snapshot: dict) -> dict:
        return { # TODO THESE NEED TUNING
            "name": self.name,
            "format": "controller",
            "input": {
                "left_stick": (snapshot["left_stick_x"], snapshot["left_stick_y"]),
                "right_stick": (snapshot["right_stick_x"], snapshot["right_stick_y"]),
                "a": snapshot["a"],
                "b": snapshot["b"],
                "x": snapshot["x"],
                "y": snapshot["y"],
                "left_bumper": snapshot["left_bumper"],
                "right_bumper": snapshot["right_bumper"],
                "left_trigger": snapshot["left_trigger"],
                "right_trigger": snapshot["right_trigger"],
                "dpad_up": snapshot["dpad_y"] == 1,
                "dpad_down": snapshot["dpad_y"] == -1,
                "dpad_left": snapshot["dpad_x"] == -1,
                "dpad_right": snapshot["dpad_x"] == 1
            }
        }


def find_controllers(handler, enabled: bool = False, priority: DevicePriority = DevicePriority.MEDIUM) -> list[Controller]:
    devices = []

    for path in list_devices():
        device = EvdevDevice(path)

        capabilities = device.capabilities()

        if (ecodes.EV_ABS in capabilities and ecodes.EV_KEY in capabilities
            and ecodes.BTN_MODE in capabilities[ecodes.EV_KEY]):
            devices.append(path)

        print("\n\n")
        print(device.name)
        print(device.capabilities(verbose=True))

    return [Controller(handler, path, enabled, None, priority) for path in devices]
    # TODO allow subclassing and determine whether to create instance of general controller or more specific one based on device name


if __name__ == "__main__":
    print(find_controllers(None))
