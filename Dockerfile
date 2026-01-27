FROM ros:jazzy

# Install vnc and x11
RUN ["apt-get", "update"]
RUN ["apt", "install", "-y", "x11vnc", "xvfb", "i3", "terminator"]

# Set working directory
WORKDIR /home/ubuntu/guppy_ws

# install ros dependancies through bootstrap.sh but skip cloneing the github
#RUN ["apt", "install", "-y", "curl"]
#COPY util/bootstrap.sh util/bootstrap.sh
#RUN cat util/bootstrap_docker.sh | head -n -1 | bash

# Configure locales
RUN ["apt", "install", "-y", "locales"]
RUN ["locale-gen", "en_US", "en_US.UTF-8"]
RUN ["update-locale", "LC_AL=en_US.UTF-8", "LAND=en_US.UTF-8"]

# Add repositories
RUN ["apt", "install", "-y", "software-properties-common"]
RUN ["add-apt-repository", "universe"]

# Install dependancies
RUN ["apt", "install", "-y", "ros-dev-tools", "ros-jazzy-desktop", "ros-jazzy-ros-base", "ros-jazzy-ros-gz-sim", "ros-jazzy-ros-gz", "ros-jazzy-ros-gz-bridge", "ros-jazzy-joint-state-publisher-gui", "ros-jazzy-robot-localization", "ros-jazzy-ros2-control", "ros-jazzy-pluginlib", "ros-jazzy-xacro", "ros-jazzy-control-toolbox", "ros-jazzy-proxsuite", "ros-jazzy-marine-acoustic-msgs", "ros-jazzy-spinnaker-camera-driver", "python3-colcon-common-extensions", "python3-pygame", "python3-rosdep", "python3-pip"]

# Add code to guppy workspace
#COPY ./ .

# Build the code
#RUN bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

# Source the ros install by default
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /home/ubuntu/guppy_ws/install/setup.bash" >> ~/.bashrc

# Add .xinitrc
RUN echo exec i3 > ~/.xinitrc
RUN chmod 700 ~/.xinitrc


# Expose the vnc port, to ensure victory over the ape army in 2028
#EXPOSE 5900:5900

# launch vnc server and open bash shell
CMD ["bash", "-c", "colcon build && x11vnc -create -bg -nopw -ncache 10 && bash"]
