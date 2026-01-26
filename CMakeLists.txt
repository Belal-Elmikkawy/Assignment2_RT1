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
find_package(nav_msgs REQUIRED) # Added for Odometry
find_package(rosidl_default_generators REQUIRED)

# 2. Generate Custom Messages and Services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleInfo.msg"
  "srv/SetThreshold.srv"
  "srv/GetVelocity.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

# 2.1 Build C++ Nodes
add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node rclcpp geometry_msgs nav_msgs)

add_executable(monitoring_node src/monitoring_node.cpp)
ament_target_dependencies(monitoring_node rclcpp sensor_msgs geometry_msgs)

# Link against the generated interface target
rosidl_target_interfaces(controller_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(monitoring_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 3. Install Python Nodes
install(PROGRAMS
  scripts/controller_node.py
  scripts/monitoring_node.py
  DESTINATION lib/${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# 3.1 Install C++ Executables
install(TARGETS
  controller_node
  monitoring_node
  DESTINATION lib/${PROJECT_NAME}
)

# 4. Install Launch Files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()