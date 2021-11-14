cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)
cmake_policy(SET CMP0054 NEW)

# Set the C++ standard we will use
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set the project name
project(ros_sim_gz)

# Set library output path to /lib
#set(LIBRARY_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/lib")

# Find Gazebo
find_package(PkgConfig REQUIRED)
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Find ROS
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

# Declare our catkin package.
catkin_package()

# Find all srcs
include_directories(include)
#file(GLOB project_SRCS src/*.cpp)

# Add plugin - shared object
set(gz_ros_node "mobicar_plugin")
add_library(${gz_ros_node} SHARED src/mobicar_plugin.cpp)
if (gazebo_FOUND)
  target_link_libraries(${gz_ros_node} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})
endif()

# Build the stand-alone ROS node program
set(vel_ros_node "rosvel")
add_executable(${vel_ros_node} src/mobicar_ros_controller_main.cpp src/mobicar_ros_controller.cpp src/stop_sign.cpp src/vehicle.cpp)
target_link_libraries(${vel_ros_node} ${roscpp_LIBRARIES} ${GAZEBO_LIBRARIES})

#install(TARGETS
#${vel_ros_node}
#${gz_ros_node}
#DESTINATION lib/${PROJECT_NAME})

install(TARGETS ${vel_ros_node}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS ${gz_ros_node}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
