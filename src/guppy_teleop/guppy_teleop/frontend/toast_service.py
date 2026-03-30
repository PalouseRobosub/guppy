import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from guppy_msgs.msg import Toast

from PySide6.QtCore import QObject

class ToastService(Node): 
    def __init__(self):
        Node.__init__(self, "toast_service")
        self.toastManager: QObject = None

        toast_quality = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(Toast, "send_toast", self._on_toast, toast_quality)
    
    def _on_toast(self, message: Toast):
        if self.toastManager is None:
            return

        self.toastManager.createMessage(self.name, {
                "type"              : message.type,
                "position"          : message.position,
                "theme"             : message.theme,
                "close_on_click"    : message.close_on_click,
                "auto_close"        : message.auto_close,
                "hide_progress_bar" : message.hide_progress_bar
            })