cmake_minimum_required(VERSION 3.5)

project(GROUPRMJ-ROBOTICS-COMPETITION)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(${PROJECT_NAME} "behavior_tree.cpp")

# Link against BehaviorTree.CPP and rclcpp
target_link_libraries(${PROJECT_NAME}
  BT::behaviortree_cpp_v3
  rclcpp::rclcpp
)

ament_target_dependencies(${PROJECT_NAME} rclcpp behaviortree_cpp_v3)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
