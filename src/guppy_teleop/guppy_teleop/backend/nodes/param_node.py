import rclpy, ast

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_event_handler import ParameterEventHandler
from rclpy.parameter_client import AsyncParameterClient
from rclpy.task import Future

from rcl_interfaces.msg import ParameterEvent
from rcl_interfaces.srv import ListParameters, GetParameters
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import SetParametersResult

from guppy_teleop.backend.nodes.node import Node as BaseNode
from guppy_teleop.backend.registry import Registry

from rclpy.logging import get_logger

logger = get_logger("guppy_teleop.param_node")

class ParameterNode(Node, BaseNode):
    @property
    def name(self) -> str:
        return "parameters"

    def __init__(self, registry):
        Node.__init__(self, "parameter_widget")
        self._registry: Registry = registry
        self._params = {}

        self.client = AsyncParameterClient(self, "control_chassis")

        self._load_parameters()

        self.handler = ParameterEventHandler(self)
        self.event_callback_handle = self.handler.add_parameter_event_callback(callback=self._on_param_change)
    
    def _load_parameters(self):
        list_client = self.create_client(ListParameters, "control_chassis/list_parameters")
        get_client = self.create_client(GetParameters, "control_chassis/get_parameters")

        if not list_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("parameter widget couldn't list control parameters!")
            return

        if not get_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("parameter widget couldn't get control parameters!")
            return

        list_request = ListParameters.Request()
        future = list_client.call_async(list_request)
        rclpy.spin_until_future_complete(self, future)

        names = future.result().result.names

        get_request = GetParameters.Request()
        get_request.names = names
        future = get_client.call_async(get_request)
        rclpy.spin_until_future_complete(self, future)

        values = future.result().values
        
        for name, value in zip(names, values):
            self._params[name] = rclpy.parameter.parameter_value_to_python(value)
    
    def get_payload(self) -> dict:
        return {"parameters": self._params}
    
    def handle_command(self, payload: dict):
        action = payload.get("action")
        arguments = payload.get("arguments", {})

        if action == "change_param":

            logger.info(f"I GOT A FREAKIN COMMAND! the action was {action}")

            param_name = arguments.get("parameter")
            type = arguments.get("type")
            value = arguments.get("value")

            self._change_param(param_name, self._convert_type(value, type))
    
    def _on_param_change(self, event: ParameterEvent):
        for param in event.changed_parameters:
            self._params[param.name] = rclpy.parameter.parameter_value_to_python(param.value)
        
        self._registry.notify(self.name, self.get_payload())
    
    def _change_param(self, name: str, value) -> bool:
        try:
            param = Parameter(name=name, value=value)

            if not self.client.wait_for_services(timeout_sec=2.0):
                raise RuntimeError("parameter service not available")
            
            future: Future[SetParametersResult] = self.client.set_parameters([param])
            rclpy.spin_until_future_complete(self, future)

            response: SetParameters.Response = future.result()

            results: list[SetParametersResult] = response.results

            for result in results:
                if not result.successful:
                    raise (Exception(result.reason))

            return True
        except Exception as err:
            logger.error(f"failed to update parameter {name}: {str(err)}!")

        return False
    
    def _convert_type(self, value, type: str):
        logger.info(type)
        match type:
            case "float":
                return float(value)
            case "list":
                parsed = ast.literal_eval(value) if isinstance(value, str) else value
                
                return [float(i) for i in parsed]
            case "str":
                return value