cmake_minimum_required(VERSION 3.5)

project(GROUPRMJ-ROBOTICS-COMPETITION)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control REQUIRED)  # Find the control package

# Add the executable for the behavior tree
add_executable(${PROJECT_NAME} "behavior_tree.cpp")

# Include directories for the control package
target_include_directories(${PROJECT_NAME} PUBLIC
  ${control_INCLUDE_DIRS}
)

# Link against BehaviorTree.CPP, rclcpp, and gripper_action_client_library
target_link_libraries(${PROJECT_NAME}
  BT::behaviortree_cpp_v3
  rclcpp::rclcpp
  gripper_action_client_library  # Link the gripper_action_client_library from control package
)

# Add dependencies for the target
ament_target_dependencies(${PROJECT_NAME} rclcpp behaviortree_cpp_v3 control)

# Install the executable
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
