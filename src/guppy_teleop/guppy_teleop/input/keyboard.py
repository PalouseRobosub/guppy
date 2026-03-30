from evdev import InputDevice as EvdevDevice, ecodes, list_devices, KeyEvent#, categorize

from geometry_msgs.msg import Twist

from guppy_teleop.input.input_device import InputDevice
from guppy_teleop.util.code_map import CodeMap
from guppy_teleop.util.device_priority import DevicePriority

from typing import Callable#, TYPE_CHECKING
#if TYPE_CHECKING:
#    from guppy_teleop.input_handler import InputHandler

class Keyboard(InputDevice):
    COMMAND_KEYS = [ecodes.KEY_LEFTCTRL, ecodes.KEY_RIGHTCTRL, ecodes.KEY_LEFTALT, ecodes.KEY_RIGHTALT]

    BLACKLISTED_DEVICES = ["ELAN2513:00 04F3:4302 Keyboard"] #("AT Translated Set 2 keyboard")

    LINEAR_MULTIPLIER = [1.0, 1.0, 1.0]
    ANGULAR_MULTIPLIER = [1.0, 1.0, 1.0]

    CODE_MAP = {
        "q": ecodes.KEY_Q,
        "w": ecodes.KEY_W,
        "e": ecodes.KEY_E,
        "a": ecodes.KEY_A,
        "s": ecodes.KEY_S,
        "d": ecodes.KEY_D,
        "o": ecodes.KEY_O,
        "p": ecodes.KEY_P,
        "up": ecodes.KEY_UP,
        "down": ecodes.KEY_DOWN,
        "left": ecodes.KEY_LEFT,
        "right": ecodes.KEY_RIGHT
    }               

    _internal_code_map = CodeMap(CODE_MAP)

    def __init__(self, handler, path: str, enabled = False, name = None, priority = DevicePriority.MEDIUM):
        self._device = EvdevDevice(path)

        super().__init__(enabled, name if name else self._device.name, priority)

        self.handler = handler

        self.COMMAND_MAP: dict[Callable, list[int]] = {
            lambda: self.handler.push_state(self, "FAULT"): [ecodes.KEY_LEFTCTRL, ecodes.KEY_SPACE],
            lambda: self.handler.push_state(self, "TELEOP"): [ecodes.KEY_LEFTCTRL, ecodes.KEY_T],
            lambda: self.handler.push_state(self, "DISABLED"): [ecodes.KEY_LEFTCTRL, ecodes.KEY_D],
            lambda: self.handler.lock_priority(self, self.priority): [ecodes.KEY_LEFTCTRL, ecodes.KEY_L],
            lambda: self.handler.lock_priority(self, None): [ecodes.KEY_LEFTCTRL, ecodes.KEY_K],
        }

        self._state = {
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

        print(f"'{self.name}' initialized!")

    async def start(self):
        async for event in self._device.async_read_loop():
            self._process(event)

    def _process(self, event):
        if event.type == ecodes.EV_KEY:
            if self._command_mode:
                active_keys = self._device.active_keys()

                if any(key in active_keys for key in self.COMMAND_KEYS):
                    for command, keys in self.COMMAND_MAP.items():
                        if all(key in active_keys for key in keys):
                            command() # remember to use try catch in your commands or it will crash with no error!

                            return

                    return
                else:
                    self._command_mode = False

                    if (event.code in self.COMMAND_KEYS):
                        return

            if (code_name := self._internal_code_map[event.code]) is not None:
                self._state[code_name] = event.value

                self._mark_active()
                if (self.handler):
                    self.handler.on_device_event(self, self._state.copy())
            else:
                if (event.code in self.COMMAND_KEYS):
                    self._command_mode = True
            
            #print(categorize(event))
            #print(f"{event.code}, {event.type}, {event.value}")

    def transform(self, snapshot: dict) -> Twist:
        twist = Twist()

        q = snapshot["q"] != KeyEvent.key_up 
        w = snapshot["w"] != KeyEvent.key_up
        e = snapshot["e"] != KeyEvent.key_up
        a = snapshot["a"] != KeyEvent.key_up
        s = snapshot["s"] != KeyEvent.key_up
        d = snapshot["d"] != KeyEvent.key_up
        o = snapshot["o"] != KeyEvent.key_up
        p = snapshot["p"] != KeyEvent.key_up
        right = snapshot["right"] != KeyEvent.key_up 
        left  = snapshot["left"]  != KeyEvent.key_up
        up    = snapshot["up"]    != KeyEvent.key_up
        down  = snapshot["down"]  != KeyEvent.key_up

        twist.linear.x = float(d - a) * self.LINEAR_MULTIPLIER[0]
        twist.linear.y = float(w - s) * self.LINEAR_MULTIPLIER[1]
        twist.linear.z = float(e - q) * self.LINEAR_MULTIPLIER[2]

        twist.angular.x = float(right - left) * self.ANGULAR_MULTIPLIER[0]
        twist.angular.y = float(up - down) * self.ANGULAR_MULTIPLIER[1]
        twist.angular.z = float(p - o) * self.ANGULAR_MULTIPLIER[2]
 
        return twist

    def package(self, snapshot: dict) -> dict:
        return {
            "name": self.name,
            "format": "keyboard",
            "input": snapshot
        }


def find_keyboards(handler, enabled: bool = False, priority = DevicePriority.MEDIUM) -> list[Keyboard]:
    devices = []

    for path in list_devices():
        device = EvdevDevice(path)

        capabilities = device.capabilities()

        if (ecodes.EV_KEY in capabilities
            and ecodes.KEY_SPACE in capabilities[ecodes.EV_KEY]
            and device.name not in Keyboard.BLACKLISTED_DEVICES):
            devices.append(device)
            #print("\n\n")
            #print(device.name)
            #print(device.capabilities(verbose=True))

    return [Keyboard(handler, path, enabled, None, priority) for path in devices]


if __name__ == "__main__":
    print(find_keyboards(None))
