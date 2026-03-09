cmake_minimum_required(VERSION 3.8)
project(bt_navigator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include)

# Executable
add_executable(bt_navigator_node src/main.cpp)

ament_target_dependencies(bt_navigator_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  behaviortree_cpp_v3
  ament_index_cpp
)

# Install
install(TARGETS bt_navigator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY trees/
  DESTINATION share/${PROJECT_NAME}/trees
)

ament_package()