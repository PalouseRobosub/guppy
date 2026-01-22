# guppy_can

Nodes and services to interface with CAN bus through ROS.

## Contents

- `can_rx`: Node that publishes all CAN frames to topics.
- `can_tx`: Service that writes data to CAN bus.

## CAN RX

The `can_rx` node publishes every frame read from the CAN bus to a topic corresponding to its CAN ID.
Topics follow the naming convention `/can/id_<can_id_hex>`, so frames with CAN ID `0x102`
will be published to the topic `/can/id_0x102`. 

### Message Type

Each CAN ID topic is created with the following `CanFrame` message type:
```
uint16 can_id
uint8 len
builtin_interfaces/Time stamp
byte[] data
```

It is imported from `guppy_msgs` and can be found at [guppy_msgs/msg/CanFrame.msg](../guppy_msgs/msg/CanFrame.msg)

### Interface Selection

The CAN interface that `can_rx` listens on is controlled by the `interface` parameter.
By default, it is set to `can0`. The interface can be hot-swapped without restarting the
node using the following command:
```shell
ros2 param set can_rx interface <can_interface_name>
# eg. ros2 param set can_rx interface can1
```

## CAN TX

The `can_tx` service writes data to CAN bus. Nodes will interface with this service directly
rather than publishing values to send data over CAN.

### Message Type

The service accepts a CAN ID and an array of up to 8 bytes. It returns the number of
bytes written to CAN bus as a conformation that the frame was sent. The `SendCan`
definition is the following:
```
uint8 id
byte[] data
---
uint8 written
```

It is imported from `guppy_msgs` and can be found at [guppy_msgs/srv/SendCan.srv](../guppy_msgs/srv/SendCan.srv)

### Interface Selection

Currently, the interface is hard-coded. It can be changed at [`src/can_tx.cpp:24`](src/can_tx.cpp#24).