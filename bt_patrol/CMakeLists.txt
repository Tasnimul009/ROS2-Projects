cmake_minimum_required(VERSION 3.8)
project(bt_patrol)

# -----------------------------------------------
# Find Dependencies
# -----------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

# -----------------------------------------------
# Include Directories
# -----------------------------------------------
include_directories(include)

# -----------------------------------------------
# Build Executable
# -----------------------------------------------
add_executable(bt_patrol_node src/main.cpp)

ament_target_dependencies(bt_patrol_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  behaviortree_cpp_v3
)

# -----------------------------------------------
# Install
# -----------------------------------------------
install(TARGETS bt_patrol_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()