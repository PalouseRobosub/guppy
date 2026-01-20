#!/bin/sh
# Builds this git repo in a docker container running ros2 with shell and VNC access

# Some stuff to find out where the Dockerfile is, not really necesarry
cd "$(dirname $0)"
cd ../

# Build and run the docker container with usb and input device access
docker build --tag guppy_docker .
docker run --rm --publish 5900:5900 --privileged -v /dev/bus/usb:/dev/bus/usb -v /dev/input:/dev/input -it guppy
