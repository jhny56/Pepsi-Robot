#ifndef GRIPPER_ACTION_CLIENT_HPP
#define GRIPPER_ACTION_CLIENT_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "robot_hardware_interfaces/action/gripper_action.hpp"

class GripperActionClient : public rclcpp::Node
{
public:
  using GripperAction = robot_hardware_interfaces::action::GripperAction;
  using GoalHandleGripperAction = rclcpp_action::ClientGoalHandle<GripperAction>;

  GripperActionClient();
  void send_goal();

private:
  rclcpp_action::Client<GripperAction>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr cancel_timer_;
  std::shared_future<GoalHandleGripperAction::SharedPtr> goal_handle_future_;
  GoalHandleGripperAction::SharedPtr goal_handle_;
  bool goal_active_;

  void goal_response_callback(GoalHandleGripperAction::SharedPtr goal_handle);
  void feedback_callback(
    GoalHandleGripperAction::SharedPtr,
    const std::shared_ptr<const GripperAction::Feedback> feedback);
  void result_callback(const GoalHandleGripperAction::WrappedResult & result);
  void cancel_goal();
};

#endif // GRIPPER_ACTION_CLIENT_HPP
