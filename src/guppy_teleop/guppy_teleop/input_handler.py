import rclpy, asyncio, threading, time

from queue import Queue

from guppy_teleop.input.input_device import InputDevice
from guppy_teleop.input.controller import find_controllers
from guppy_teleop.input.keyboard import find_keyboards
from guppy_teleop.util.device_priority import DevicePriority

from geometry_msgs.msg import Twist

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from typing import Callable

class InputHandler(Node):
    TIMEOUT = 0.5

    def __init__(self, update_widget_callback: Callable = None):
        super().__init__("teleop_input")

        self._update_widget_callback = update_widget_callback

        self.queue = Queue()

        self.input_thread: threading.Thread = None

        self.input_devices: list[InputDevice] = []

        self._focus: InputDevice = None

        self._priority_lock = None

        quality = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.publisher = self.create_publisher(Twist, "/cmd_vel/teleop", quality)

        self.timer = self.create_timer(0.05, self._watchdog)
    
    def thread_start(self):
        self.input_thread = threading.Thread(target=lambda: asyncio.run(self.start_devices()), daemon=True)
        self.input_thread.start()

    async def start_devices(self):
        tasks = [asyncio.create_task(device.start()) for device in self.input_devices]

        if len(tasks) == 0:
            return
        
        try:
            await asyncio.wait(tasks, return_when=asyncio.FIRST_EXCEPTION)
        except asyncio.CancelledError:
            pass
        finally:
            for task in tasks:
                task.cancel()

            await asyncio.gather(*tasks, return_exceptions=True)

    def on_device_event(self, device: InputDevice, snapshot: dict): # passes snapshot of state to prevent race conditions of reading the active state
        if not device.active: # see thread-safe issue on _watchdog(), has the potential to eat inputs, possibly safe to remove condition as by recieving a event means the device is "active" and can have last_time set to the current time?
            return
        if self._priority_lock is not None and device.priority < self._priority_lock:
            return
        elif self._focus is None:
            self._focus = device
        elif device.priority > self._focus.priority:
            self._focus = device
        else:
            return

        if (self._update_widget_callback):
            self._update_widget_callback(self._focus.package(snapshot))

        self.publisher.publish(self._focus.transform(snapshot))

    # this guy is NOT thread-safe and can cause a race-condition while attempting to mark the device inactive HOWEVER
    # that's okkkkkk. a zero twist message will STILL be published (preventing hanging) and focus will be lost essentially
    # eating a single input which kinda sucks but might happen. just call it a skill issue.
    def _watchdog(self):
        if self._focus:
            dt = time.time() - self._focus.last_active
            if dt > self.TIMEOUT:
                self._focus._mark_inactive() #TODO someone smarter than me make this work better
                self._focus = None

                self.publisher.publish(Twist())

    def add_device(self, *devices: InputDevice):
        for device in devices:
            device.handler = self

            self.input_devices.append(device)
    
    def lock_priority(self, priority: DevicePriority):
        self._priority_lock = priority
        print(f"priority locked to '{priority}'!")
    
    def push_state(self, state: str):
        print(f"sending '{state}'!")


def main(args=None):
    rclpy.init()

    devices: list[InputDevice] = []

    devices.extend(find_controllers(True))
    devices.extend(find_keyboards(True))

    handler = InputHandler(None)

    handler.add_device(*devices)

    handler.thread_start()

    try:
        rclpy.spin(handler)
    finally:
        handler.destroy_node()
        rclpy.shutdown()


def controller(args=None):
    rclpy.init()

    devices: list[InputDevice] = []

    devices.extend(find_controllers(True))

    handler = InputHandler(None) # TODO

    handler.add_device(*devices)

    handler.thread_start()

    try:
        rclpy.spin(handler)
    finally:
        handler.destroy_node()
        rclpy.shutdown()


def keyboard(args=None):
    rclpy.init()

    devices: list[InputDevice] = []
    devices.extend(find_keyboards(True))

    handler = InputHandler(None)
    handler.add_device(*devices)
    handler.thread_start()

    try:
        rclpy.spin(handler)
    finally:
        handler.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
