#ifndef GRIPPER_ACTION_SERVER_HPP_
#define GRIPPER_ACTION_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <memory>
#include <thread>

class GripperActionServer : public rclcpp::Node {
public:
  using GripperCommand = control_msgs::action::GripperCommand;
  using GoalHandleGripperCommand = rclcpp_action::ServerGoalHandle<GripperCommand>;

  explicit GripperActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp_action::Server<GripperCommand>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GripperCommand::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleGripperCommand> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleGripperCommand> goal_handle);

  void execute(const std::shared_ptr<GoalHandleGripperCommand> goal_handle);

  void close_gripper(double position, double max_effort);

  void open_gripper();

  double get_current_position();

  double get_current_effort();
};

#endif  // GRIPPER_ACTION_SERVER_HPP_
