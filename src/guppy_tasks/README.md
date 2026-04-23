# Align Node

Aligns the AUV to an AprilTag using a P controller.

## Launch

```bash
ros2 launch guppy_vision core.xml
```

## Usage

```bash
ros2 action send_goal --feedback /align guppy_msgs/action/Align \
  "{tag_name: 'tag_0', target_distance: 1.0}"
```

Replace `tag_0` with the target tag ID and `target_distance` with the desired standoff distance in meters.