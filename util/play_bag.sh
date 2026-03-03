#!/bin/bash

# Replays the ros bag saved as guppy
ros2 bag info guppy_recording
ros2 bag play guppy_recording
