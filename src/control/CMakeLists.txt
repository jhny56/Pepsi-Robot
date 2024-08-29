cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_hardware_interfaces REQUIRED)

# Include directories
include_directories(
  include
)

# Add the executable for the action client
add_executable(gripper_action_client src/gripper_action_client.cpp)
ament_target_dependencies(gripper_action_client rclcpp rclcpp_action control_msgs robot_hardware_interfaces)

# Install the executable for the action client
install(TARGETS gripper_action_client
  DESTINATION lib/${PROJECT_NAME}
)

# Add the executable for initNode
add_executable(initNode src/initNode.cpp)
ament_target_dependencies(initNode rclcpp rclpy)

# Install the executable for initNode
install(TARGETS initNode
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Enable testing and linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
