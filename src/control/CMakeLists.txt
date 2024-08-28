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
  ${robot_hardware_interfaces_INCLUDE_DIRS}  # Add this line
)

# Add the executable for the action server
add_executable(gripper_action_server src/gripper_action_server_main.cpp src/gripper_action_server.cpp)
ament_target_dependencies(gripper_action_server rclcpp rclcpp_action control_msgs)

# Create a library for the gripper action client
add_library(gripper_action_client_library SHARED src/gripper_action_client.cpp)
target_include_directories(gripper_action_client_library PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(gripper_action_client_library rclcpp rclcpp_action control_msgs robot_hardware_interfaces)

# Install the gripper action client library
install(TARGETS gripper_action_client_library
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the executable for the action server
install(TARGETS gripper_action_server
  DESTINATION lib/${PROJECT_NAME}
)

# Add the executable for initNode
add_executable(initNode src/initNode.cpp)
target_include_directories(initNode PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(initNode PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(initNode rclcpp rclpy)

# Install the executable for initNode
install(TARGETS initNode
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Enable testing and linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The following line skips the linter which checks for copyrights
  # Comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # The following line skips cpplint (only works in a git repo)
  # Comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
