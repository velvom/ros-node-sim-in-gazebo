# Set the base image.
# Since we're running a ros application, a ros base image is used.
FROM ros:melodic

# Set a key-value label for the Docker image
LABEL maintainer="Navamshan Murugesapillai"

# Install ros package and required software to open GUI in the container
RUN apt-get update && apt-get install -y \
    ros-melodic-gazebo-ros-pkgs \
    python3-colcon-common-extensions \
    x11vnc \
    xvfb \
    openbox

# Optimize by deleting unneeded apt lists
RUN rm -rf /var/lib/apt/lists/*

# Prepare to open GUI in the container
RUN mkdir ~/.vnc

# Windowing for terminal gazebo window
RUN echo "openbox &" >> /.bashrc

# Start roscore
RUN echo "roscore &" >> /.bashrc
# Give ample time for ROS master to be up and running
RUN echo "sleep 5" >> /.bashrc

# cd to the project workspace and build it
RUN echo "cd /home/rossim && colcon build" >> /.bashrc
# source the built packages to be able to use them
RUN echo "source install/setup.bash" >> /.bashrc

# Start ROS-aware Gazebo node
RUN echo "source /usr/share/gazebo/setup.sh" >> /.bashrc
ENV GAZEBO_MODEL_PATH="/home/rossim/src/ros_sim_gz/models:${GAZEBO_MODEL_PATH}"
RUN echo "cd /home/rossim && \
    rosrun gazebo_ros gazebo --verbose src/ros_sim_gz/mobicar.world &" >> /.bashrc

# Start the external ROS node to begin the simulation.
RUN echo "rosrun ros_sim_gz rosvel" >> /.bashrc
