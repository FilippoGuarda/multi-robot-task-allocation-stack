cmake_minimum_required(VERSION 3.5)
project(social_navigation_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set (msg_files
  "msg/HumanState.msg"
  "msg/HumanStates.msg"
  "msg/HumanClosestObstacle.msg"
  "msg/HumanClosestObstacles.msg"
  "msg/RobotClosestObstacle.msg"
  "msg/RobotClosestObstacles.msg"
  "msg/Feedback.msg"
  "msg/PoseStampedPair.msg"
  "msg/PathRequest.msg"
  "msg/PathReturn.msg"
  "msg/RobotCluster.msg"
  "msg/QueueRequest.msg"
  "msg/QueueMsg.msg"
  "msg/Plan.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()