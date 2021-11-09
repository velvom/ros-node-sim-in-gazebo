cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)

# Set the C++ standard we will use
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set the project name
project(mobicar_plugin)

# Set library output path to /lib
set(LIBRARY_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/lib")

# Find ROS
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Find all srcs
include_directories(include)
#file(GLOB project_SRCS src/*.cpp)

# Add plugin - shared object
add_library(${PROJECT_NAME} SHARED src/mobicar_plugin.cpp)
if (gazebo_FOUND)
  target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})
endif()

# Build the stand-alone ROS node program
add_executable(rosvel src/mobicar_ros_controller_main.cpp src/mobicar_ros_controller.cpp src/stop_sign.cpp src/vehicle.cpp)
target_link_libraries(rosvel ${roscpp_LIBRARIES} ${GAZEBO_LIBRARIES})
