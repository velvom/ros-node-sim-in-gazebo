# ROS Node Simulated in Gazebo

<img src="data/rosnode_simulation_gazebo.gif"/>

This includes the following:
* an SDF model of a mobile car named “nava_mobicar” for Gazebo.
* a ROS-aware Gazebo plugin to control the model simulation in Gazebo.
* a custom .world file that attaches the plugin to nava_mobicar model and includes other surrounding elements such as light source, ground plane, stop sign, tree, etc.
* source code for the main ROS node that communicates with Gazebo-ROS node

## Dependencies for Running Locally
* cmake >= 2.8
  * Linux OS: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux)
  * Linux: make is installed by default on most Linux distros
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
* Install GAZEBO, includiing headers needed for plugin development:
  * Linux OS: [click here for installation instructions](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)
* Install ROS: [click here for installation instructions](http://wiki.ros.org/ROS/Installation)
  * For instance, for Ubuntu 20.04 LTS, install ROS Noetic Ninjemys distribution.
* Install ros_control: [click here for installation instructions](http://wiki.ros.org/ros_control)
  * For Notetic distribution: sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Source ROS: source /opt/ros/noetic/setup.bash
4. Compile: `cmake .. && make`

## Running Instructions

1. From a termial, source ROS and start roscore as follows.  
   $ `source /opt/ros/noetic/setup.bash`  
   $ `roscore`

2. Open another terminal and execute the following from the top-level directory to launch Gazebo world with mobicar model.  
   $ `cd build`  
   $ `source /usr/share/gazebo-11/setup.sh`  
   $ `export GAZEBO_MODEL_PATH=<top-level directory>/models:${GAZEBO_MODEL_PATH}`  
   $ `rosrun gazebo_ros gazebo --verbose ../mobicar.world`

3. Now, from the terminal you compiled the code, execute the main ros node.  
   $ `cd <top-level directory>/build`  
   $ `./rosvel`
