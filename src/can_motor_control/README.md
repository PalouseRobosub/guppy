# WIP - NOT UP TO DATE

# can_motor_control
Publishes and sends throttle values over CAN bus in ROS2 Jazzy Jalisco

## Prerequisites
- ROS2 Jazzy Jalisco installed on Ubuntu 24.04LTS
- `can-utils` installed
- CAN or VCAN network interface configured
- Correct CAN or VCAN network set in [src/can_throttle_subscriber.cpp](src/can_throttle_subscriber.cpp) (Line 67)

## Getting Started

Open a new terminal and source the ROS2 environment:
```bash
source /opt/ros/jazzy/setup.bash
```

### Create a ROS2 Workspace

Create a workspace directory and navigate into it:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### Clone the Repository

Clone the repository into the workspace:
```bash
git clone https://github.com/asorge29/ros2_can_motor_control.git
```

### Install Dependencies

Install the required dependencies:
```bash
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro jazzy -y
```

### Build the Package

This will build *only* the `can_motor_control` package:
```bash
colcon build --packages-select can_motor_control
```

### Run the Publisher Node

First, source the setup file:
```bash
source ~/ros2_ws/install/setup.bash
```

Then, run the publisher node:
```bash
ros2 run can_motor_control throttle_publisher
```

### Run the Subscriber Node

Open a new terminal and source the ROS2 environment:
```bash
source /opt/ros/jazzy/setup.bash
```

Source the setup file:
```bash
source ~/ros2_ws/install/setup.bash
```

Then, run the subscriber node:
```bash
ros2 run can_motor_control can_throttle_subscriber
```

## Usage

The publisher node will take user input for a float between `-1.0` and `1.0`.
It will immediately publish it, and the subscriber will send it over CAN with the id `0x101`.

To verify that the frames are correctly being sent over CAN, run in a seperate terminal:
```bash
candump can0 # replace can0 with your CAN interface