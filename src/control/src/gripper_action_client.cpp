#include "../include/control/gripper_action_client.hpp"

GripperActionClient::GripperActionClient()
: Node("gripper_action_client"), goal_active_(false)
{
  this->client_ = rclcpp_action::create_client<GripperAction>(this, "gripper_action");
  this->send_goal();
}

void GripperActionClient::send_goal()
{
  if (!this->client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    return;
  }

  auto goal_msg = GripperAction::Goal();
  // Set any goal parameters if required

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<GripperAction>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&GripperActionClient::goal_response_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&GripperActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&GripperActionClient::result_callback, this, std::placeholders::_1);

  this->goal_handle_future_ = this->client_->async_send_goal(goal_msg, send_goal_options);


}

void GripperActionClient::goal_response_callback(GoalHandleGripperAction::SharedPtr goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    goal_active_ = false;
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    goal_handle_ = goal_handle;
    goal_active_ = true;
  }
}

void GripperActionClient::feedback_callback(
  GoalHandleGripperAction::SharedPtr,
  const std::shared_ptr<const GripperAction::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Received feedback: %d", feedback->feedback);
}

void GripperActionClient::result_callback(const GoalHandleGripperAction::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(this->get_logger(), "Goal was canceled");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      break;
  }

  RCLCPP_INFO(this->get_logger(), "Result: %d", result.result->result);

  goal_active_ = false;
}

void GripperActionClient::cancel_goal()
{
  if (goal_handle_ && goal_active_) {
    RCLCPP_INFO(this->get_logger(), "Attempting to cancel the goal after 30 seconds");
    this->client_->async_cancel_goal(goal_handle_);
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal is already completed or no valid goal handle exists.");
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperActionClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
