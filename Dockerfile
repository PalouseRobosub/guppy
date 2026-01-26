FROM ros:jazzy-perception

# Install vnc and x11
RUN ["apt-get", "update"]
RUN ["apt", "install", "-y", "x11vnc", "xvfb"]

# Set working directory
WORKDIR /home/ubuntu/guppy_ws

# install ros dependancies through bootstrap.sh but skip cloneing the github
RUN ["apt", "install", "-y", "curl"]
RUN curl https://raw.githubusercontent.com/PalouseRobosub/guppy/refs/heads/main/util/bootstrap.sh | head -n +1 | bash

# hotfix as there is a missing package in bootstrap script
RUN ["apt", "install", "-y", "python3-pygame"]

# Add code to guppy workspace
COPY ./ .

# Build the code
RUN bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

# Source the guppy install by default
RUN echo "source /home/ubuntu/guppy_ws/install/setup.bash" >> ~/.bashrc

# Expose the vnc port, to ensure victory over the ape army in 2028
#EXPOSE 5900:5900

# launch vnc server and open bash shell
CMD ["bash", "-c", "x11vnc -create -bg -nopw -ncache 10 && bash"]
