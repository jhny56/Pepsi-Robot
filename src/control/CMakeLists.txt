cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_hardware_interfaces REQUIRED)

# Include directories
include_directories(include)

# Add the executable for the action server
add_executable(gripper_action_server src/gripper_action_server_main.cpp src/gripper_action_server.cpp)
add_executable(gripper_action_client src/gripper_action_client.cpp)

# Link dependencies to the executable
ament_target_dependencies(gripper_action_server rclcpp rclcpp_action control_msgs)
ament_target_dependencies(gripper_action_client rclcpp rclcpp_action control_msgs robot_hardware_interfaces)


# Install the executable
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(TARGETS
  gripper_action_server
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS
  gripper_action_client
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(initNode src/initNode.cpp)
target_include_directories(initNode PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(initNode PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  initNode
  "rclcpp"
  "rclpy"
)

install(TARGETS initNode
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
