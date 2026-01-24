cmake_minimum_required(VERSION 3.8)
project(assignment2_rt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find all dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 2. Generate the Custom Messages and Services
# Ensure these filenames match exactly what is in your msg/ and srv/ folders
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleInfo.msg"
  "srv/SetThreshold.srv"
  "srv/GetVelocity.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

# 2.1 Build C++ Nodes
# 2.1 Build C++ Nodes
add_executable(controller_node_cpp src/controller_node.cpp)
ament_target_dependencies(controller_node_cpp rclcpp geometry_msgs sensor_msgs)
rosidl_target_interfaces(controller_node_cpp ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(monitoring_node_cpp src/monitoring_node.cpp)
ament_target_dependencies(monitoring_node_cpp rclcpp geometry_msgs sensor_msgs)
rosidl_target_interfaces(monitoring_node_cpp ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install C++ Executables
install(TARGETS
  controller_node_cpp
  monitoring_node_cpp
  DESTINATION lib/${PROJECT_NAME})


# 3. Install the Python Nodes
# This copies your scripts to lib/assignment2_rt/ so "ros2 run" works
install(PROGRAMS
  scripts/controller_node.py
  scripts/monitoring_node.py
  DESTINATION lib/${PROJECT_NAME})


# 4. Install Launch Files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
