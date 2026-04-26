source install/setup.bash
ros2 action send_goal --feedback /navigate guppy_msgs/action/Navigate "{pose: {position: {x: 1, y: 0.0, z: 0.0}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}, local: false, duration: 5}"
ros2 action send_goal --feedback /navigate guppy_msgs/action/Navigate "{pose: {position: {x: 1, y: 0.5, z: 0.0}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}, local: false, duration: 5}"
ros2 action send_goal --feedback /navigate guppy_msgs/action/Navigate "{pose: {position: {x: 1, y: -0.5, z: 0.0}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}, local: false, duration: 5}"
ros2 action send_goal --feedback /navigate guppy_msgs/action/Navigate "{pose: {position: {x: 1, y: 0.0, z: 0.0}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}, local: false, duration: 5}"
ros2 action send_goal --feedback /navigate guppy_msgs/action/Navigate "{pose: {position: {x: 0, y: 0.0, z: 0.0}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}, local: false, duration: 5}"
