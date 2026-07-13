```
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -r /image_raw:=/cube/image_raw \
  -p video_device:=/dev/video0 \
  -p image_width:=640 \
  -p image_height:=480 \
  -p pixel_format:=mjpeg2rgb \
  -p framerate:=30.0
```

```
ros2 run guppy_tasks lanefollowaction
```


```
ros2 action list
ros2 action info /lane_follow
ros2 action send_goal /lane_follow guppy_msgs/action/LaneFollow "{lost_timeout: 2.0}" --feedback
```

```
ros2 run rqt_image_view rqt_image_view
```

```
ros2 param set /lane_navigator_py hsv_red1_low "[0, 100, 100]"
ros2 param set /lane_navigator_py hsv_red1_high "[10, 255, 255]"
ros2 param set /lane_navigator_py hsv_white_low "[0, 0, 55]"
ros2 param set /lane_navigator_py hsv_white_high "[179, 45, 255]"
ros2 param set /lane_navigator_py min_aspect 0.3
ros2 param set /lane_navigator_py min_contour_area 100
```