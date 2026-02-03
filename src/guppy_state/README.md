# guppy_state

Node, service, and message for managing the guppy state machine over ROS.

### States

The following states are expected by the `change_state` service,

- INITIAL
- HOLDING
- NAV
- TASK
- TELEOP
- DISABLED
- FAULT

They are represented by 8-bit unsigned integers and published to the `/state` topic. When reading this state, use the constants provided by the `state` message for comparison as mentioned below.

### Service Definition

Use the `change_state` service like [this tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html#write-the-client-node) to format a request according to the definition:

```go
guppy_msgs/State state
---
bool success
```

This can be imported from [../guppy_msgs/srv/ChangeState.srv](../guppy_msgs/srv/ChangeState.srv) as `guppy_msgs::srv::ChangeState::Request` in C++ or `from guppy_msgs.srv import ChangeState` in Python.

### Constants

Defined in the `state` message from are the following constants.

```go
uint8 INITIAL   = 0
uint8 HOLDING   = 1
uint8 NAV       = 2
uint8 TASK      = 3
uint8 TELEOP    = 4
uint8 DISABLED  = 5
uint8 FAULT     = 6
```

Use these to create a State message and ChangeState request to ensure you set the correct state. They are defined in [../guppy_msgs/msg/State.msg](../guppy_msgs/msg/State.msg) and are included. The startup state, for example, is represented by `guppy_msgs::msg::State::STARTUP` in C++ or `State.STARTUP` in Python.

### Transitioning

Transitioning between states is handled by the state manager according to the graph below. Importantly, the `STARTUP` state is not reachable from any other state and once the `FAULT` state is hit there is no way to transition to another state. The service will return a `true` if transitioning to the state was successful and `false` otherwise.

<img width="500" height="400" alt="image" src="https://github.com/user-attachments/assets/50d61a56-c202-4307-8e1a-609c3002b127"/>

### Using `/cmd_vel`

The `/cmd_vel` topic is published to by the state manager by forwarding the twist messages of from the subtopic of the active state, ie. guppy_teleop now published to the `/cmd_vel/teleop` topic and, provided guppy is in the TELEOP state, the state manager will forward these messages to `/cmd_vel`.

### Change State Example

After creating a node,

```cpp
rclcpp::Client<guppy_msgs::srv::ChangeState>::SharedPtr client = node->create_client<guppy_msgs::srv::ChangeState>("change_state");
```

Then format a request using the State message from [../guppy_msgs/msg/State.msg](../guppy_msgs/msg/State.msg).

```cpp
auto message = guppy_msgs::msg::State();
message.state = guppy_msgs::msg::State::NAV;

auto request = std::make_shared<guppy_msgs::srv::ChangeState::Request>();
request->new_state = message;

auto result = client->async_send_request(request);

if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    // SUCCESS
} else {
    // FAIL
}
```

Refer to [this ROS2 tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html) for more information.