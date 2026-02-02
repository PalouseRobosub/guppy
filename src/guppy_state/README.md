# guppy_state

Node and service for managing the guppy state machine over ROS.

### States

The following states are accepted by the service:

- INITIAL
- HOLDING
- NAV
- TASK
- TELEOP
- DISABLED
- FAULT

They are represented by 8-bit unsigned integers and published to the `/state` topic. When reading this state, use the constants provided by the service for comparison as mentioned below.

### Service Definition

Use the `change_state` service like [this tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html#write-the-client-node) to format a request according to the definition:

```
uint8 state
---
bool success
```

This can be imported from [../guppy_msgs/srv/ChangeState.srv](../guppy_msgs/srv/ChangeState.srv) as `guppy_msgs::srv::ChangeState::Request` in C++ or `from guppy_msgs.srv import ChangeState` in Python.

### Constants

Defined alongside the service are the following constants. Use this to format a request to ensure you set the correct state. The initial state for example is represented by, `guppy_msgs::src::ChangeState::Request::INITIAL` in C++ or `ChangeState.Request.INITIAL` in Python.

```
uint8 INITIAL   = 0
uint8 HOLDING   = 1
uint8 NAV       = 2
uint8 TASK      = 3
uint8 TELEOP    = 4
uint8 DISABLED  = 5
uint8 FAULT     = 6
```

### Transitioning

Transitioning between states is handeled by the state manager according to the graph below. Importantly, the INITIAL state is not reachable from any other state and once the FAULT state is hit there is no way to transition to another state. The service will return a boolean if transitioning to the state was succeful.

<img width="500" height="400" alt="image" src="https://github.com/user-attachments/assets/50d61a56-c202-4307-8e1a-609c3002b127"/>

### Using `/cmd_vel`

The `/cmd_vel` topic is published to by the state manager by forwarding the twist messages of from the subtopic of the active state, ie. gupp_teleop now published to the `/cmd_vel/teleop` topic and, provided guppy is in the TELEOP state, the state manager will forward these messages to `/cmd_vel`.