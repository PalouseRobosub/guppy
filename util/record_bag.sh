#!/bin/bash

# Records everything posted to ros namespace into a ros bag named guppy.bag

# Delete the old rosbag
cd "$(dirname $0)"
cd ../
rm -rf guppy_recording

# Create the new
ros2 bag record -o guppy_recording -a
