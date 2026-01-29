FROM ros:jazzy

RUN apt-get update

# Configure locales
RUN apt install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add repositories
RUN apt install -y software-properties-common
RUN add-apt-repository universe

# Install dependancies
RUN apt install -y \
    ros-dev-tools \
    ros-jazzy-desktop \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-localization \
    ros-jazzy-ros2-control \
    ros-jazzy-pluginlib \
    ros-jazzy-xacro \
    ros-jazzy-control-toolbox \
    ros-jazzy-proxsuite \
    ros-jazzy-marine-acoustic-msgs \
    ros-jazzy-spinnaker-camera-driver \
    python3-colcon-common-extensions \
    python3-pygame \
    python3-rosdep \
    python3-pip

# Install extra utilities
RUN apt install -y \
    net-tools \
    can-utils

# VNC Setup
RUN apt update && apt install -y git python3 python3-websockify && \
    git clone https://github.com/novnc/noVNC.git /opt/novnc

# DE setup
RUN apt-get install -y \
    xfce4 xfce4-goodies \
    tigervnc-standalone-server tigervnc-common tigervnc-tools \
    dbus-x11 terminator \
    wget

RUN mkdir -p /root/.vnc && \
    printf '#!/bin/sh\nunset SESSION_MANAGER\nunset DBUS_SESSION_BUS_ADDRESS\nexec startxfce4\n' \
    > /root/.vnc/xstartup && \
    chmod +x /root/.vnc/xstartup

RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# Clone the sim
RUN git clone https://github.com/PalouseRobosub/GNCea.git ~/GNCea

# Source the ros install by default
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc