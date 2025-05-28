cmake_minimum_required(VERSION 3.15)
project(rwr-ros2-examples)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall)


find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

## Find system libraries
find_package(tf2_eigen REQUIRED)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  # Set because Boost is an internal dependency, not transitive.
  ${fmt_INCLUDE_DIRS}
)


add_executable(joy_to_twist
  src/joy_to_twist.cpp
)
ament_target_dependencies(joy_to_twist rclcpp std_msgs sensor_msgs tf2_eigen)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS 
  joy_to_twist
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
    scripts/franka_pose_cmd_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# install the launch file
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()

