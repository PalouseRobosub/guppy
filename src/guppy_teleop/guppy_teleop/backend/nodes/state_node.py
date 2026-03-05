import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from guppy_msgs.msg import State
from guppy_msgs.srv._change_state import ChangeState

from guppy_teleop.backend.registry import Registry
from guppy_teleop.backend.nodes.node import Node as BaseNode

from enum import Enum

class ValidState(Enum):
    STARTUP = 0
    HOLDING = 1
    NAV = 2
    TASK = 3
    TELEOP = 4
    DISABLED = 5
    FAULT = 6

class StateNode(Node, BaseNode):
    @property
    def name(self) -> str:
        return "state"

    def __init__(self, registry):
        Node.__init__(self, "state_widget")
        self._state: str = "no state recieved"
        self._registry: Registry = registry

        self._client = self.create_client(ChangeState, "change_state")
        self.req = None
        if not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("chage_state service not available")
        else:
            self.req = ChangeState.Request()

        state_quality = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(State, "state", self._on_state, state_quality)
    
    def get_payload(self) -> dict:
        return {"state": self._state}
    
    def handle_command(self, payload: dict):
        action = payload.get("action")
        arguments = payload.get("arguments")

        if action == "change_state":
            state = arguments.get("new_state")

            self._change_state(state)
    
    def _on_state(self, message: State):
        try:
            self._state = ValidState(message.state).name
        except ValueError:
            self._state = "UNKNOWN"

        self._registry.notify(self.name, self.get_payload())
    
    def _change_state(self, new_state: str) -> bool:
        message = State()
        if (valid_state := getattr(ValidState, new_state, None)) is None:
            return False
        
        message.state = valid_state.value
        self.req.new_state = message

        future = self._client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success
