import json

from guppy_teleop.backend.nodes.node import Node

class Registry:
    def __init__(self, broadcast_callback):
        self._nodes: dict[str, Node] = {}
        self._broadcast = broadcast_callback
    
    def register(self, node):
        self._nodes[node.name] = node
    
    def notify(self, name: str, payload: dict):
        if self._broadcast is None:
            return
        
        self._broadcast({
            "type": "update",
            "name": name,
            "payload": payload
        })
    
    def sync_client(self):
        if self._broadcast is None:
            return
        
        for name, node in self._nodes.items():
            self._broadcast({
                "type": "sync",
                "name": name,
                "payload": node.get_payload()
            })
    
    def route(self, message: dict):
        name = message.get("name")
        payload = message.get("payload", {})

        node = self._nodes.get(name)
        if node is None:
            return
        
        node.handle_command(payload)

    def send_command(self, name: str, action: str, arguments=None):
        if self._broadcast is None:
            return
        
        self._broadcast({
            "type": "command",
            "name": name,
            "payload": {"action": action, "arguments": arguments}
        })