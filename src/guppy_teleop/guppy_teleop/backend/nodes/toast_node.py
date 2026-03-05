import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from guppy_msgs.msg import Toast

from guppy_teleop.backend.registry import Registry
from guppy_teleop.backend.nodes.node import Node as BaseNode

class ToastNode(Node, BaseNode):
    @property
    def name(self) -> str:
        return "toast"

    def __init__(self, registry):
        Node.__init__(self, "toast_widget")
        self._registry: Registry = registry

        toast_quality = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.create_subscription(Toast, "send_toast", self._on_toast, toast_quality)
    
    def _on_toast(self, message: Toast):
        self._registry.send_command(self.name, "send_toat", {
                "type"              : message.type,
                "position"          : message.position,
                "theme"             : message.theme,
                "close_on_click"    : message.close_on_click,
                "auto_close"        : message.auto_close,
                "hide_progress_bar" : message.hide_progress_bar
            })
        
    
    def get_payload(self) -> dict:
        return {}
    
    def handle_command(self, payload: dict):
        pass