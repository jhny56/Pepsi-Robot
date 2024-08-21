#include "control/gripper_action_server.hpp"

GripperActionServer::GripperActionServer(const rclcpp::NodeOptions & options)
: Node("gripper_action_server")
{
  RCLCPP_INFO(this->get_logger(), "Initializing Gripper Action Server...");

  this->action_server_ = rclcpp_action::create_server<GripperCommand>(
    this,
    "gripper_command",
    std::bind(&GripperActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&GripperActionServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&GripperActionServer::handle_accepted, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse GripperActionServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const GripperCommand::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request ");
  (void)uuid;
  // position is between 0 and 1 (1 means it is fully closed)
  // max_effort is the maximum force to apply, so to avoid crushing the object we would need a lower value
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GripperActionServer::handle_cancel(
  const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  open_gripper();  // Open the gripper when the goal is canceled
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GripperActionServer::handle_accepted(const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Goal accepted, starting execution.");
  std::thread{std::bind(&GripperActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void GripperActionServer::execute(const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<GripperCommand::Result>();
  auto feedback = std::make_shared<GripperCommand::Feedback>();

  // Close the gripper as per the goal request
  close_gripper(goal->command.position, goal->command.max_effort);
  RCLCPP_INFO(this->get_logger(), "Gripper closed to position %f with max effort %f", goal->command.position, goal->command.max_effort);

  // Wait for 1 minute while monitoring for cancellation
  rclcpp::Time start_time = this->now();
  rclcpp::Duration duration = rclcpp::Duration::from_seconds(60);
  rclcpp::Rate rate(10);  // Check every 100ms

  while (this->now() - start_time < duration) {
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "Goal canceled during execution, opening gripper immediately.");
      open_gripper();  // Open the gripper immediately if canceled
      result->position = get_current_position();
      result->effort = get_current_effort();
      result->reached_goal = false;
      goal_handle->canceled(result);
      return;
    }

    // Update feedback
    feedback->position = get_current_position();
    feedback->effort = get_current_effort();
    feedback->reached_goal = false;
    feedback->stalled = false;
    goal_handle->publish_feedback(feedback);

    rate.sleep();
  }

  // After 1 minute, open the gripper
  open_gripper();
  RCLCPP_INFO(this->get_logger(), "Gripper opened after 1 minute.");

  // Set the result
  result->position = get_current_position();
  result->effort = get_current_effort();
  result->reached_goal = true;
  result->stalled = false;
  goal_handle->succeed(result);

  RCLCPP_INFO(this->get_logger(), "Goal succeeded");
}

void GripperActionServer::close_gripper(double position, double max_effort)
{
  // Add logic to close the gripper
  RCLCPP_INFO(this->get_logger(), "Closing gripper to position %f with max effort %f", position, max_effort);
}

void GripperActionServer::open_gripper()
{
  // Add logic to open the gripper
  RCLCPP_INFO(this->get_logger(), "Opening gripper...");
}

double GripperActionServer::get_current_position()
{
  // Add logic to get the current position of the gripper
  return 0.0;  // Replace with actual position
}

double GripperActionServer::get_current_effort()
{
  // Add logic to get the current effort of the gripper
  return 0.0;  // Replace with actual effort
}
