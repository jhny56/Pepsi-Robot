#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "control_msgs/action/gripper_command.hpp"

class GripperActionClient : public rclcpp::Node
{
public:
  using GripperCommand = control_msgs::action::GripperCommand;
  using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;

  GripperActionClient()
  : Node("gripper_action_client"), goal_active_(false)
  {
    this->client_ = rclcpp_action::create_client<GripperCommand>(this, "gripper_command");

    this->send_goal();
  }

  void send_goal()
  {
    if (!this->client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    auto goal_msg = GripperCommand::Goal();
    goal_msg.command.position = 0.5;  // Set desired position (e.g., halfway closed)
    goal_msg.command.max_effort = 10.0;  // Set desired max effort

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&GripperActionClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(&GripperActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&GripperActionClient::result_callback, this, std::placeholders::_1);

    this->goal_handle_future_ = this->client_->async_send_goal(goal_msg, send_goal_options);

    // Set a timer to cancel the goal after 30 seconds
    this->cancel_timer_ = this->create_wall_timer(
      std::chrono::seconds(30),
      std::bind(&GripperActionClient::cancel_goal, this)
    );
  }

private:
  rclcpp_action::Client<GripperCommand>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr cancel_timer_;
  std::shared_future<GoalHandleGripperCommand::SharedPtr> goal_handle_future_;
  GoalHandleGripperCommand::SharedPtr goal_handle_;
  bool goal_active_;

  void goal_response_callback(GoalHandleGripperCommand::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      goal_active_ = false;
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      goal_handle_ = goal_handle;  // Store the valid goal handle
      goal_active_ = true;  // Mark goal as active
    }
  }

  void feedback_callback(
    GoalHandleGripperCommand::SharedPtr,
    const std::shared_ptr<const GripperCommand::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Received feedback: position = %f, effort = %f",
      feedback->position, feedback->effort);
  }

  void result_callback(const GoalHandleGripperCommand::WrappedResult & result)
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

    RCLCPP_INFO(this->get_logger(), "Result: position = %f, effort = %f, reached_goal = %d, stalled = %d",
      result.result->position, result.result->effort, result.result->reached_goal, result.result->stalled);

    goal_active_ = false;  // Mark goal as inactive
  }

  void cancel_goal()
  {
    // Ensure that the goal handle is valid and the goal is still active before attempting to cancel
    if (goal_handle_ && goal_active_) {
      RCLCPP_INFO(this->get_logger(), "Attempting to cancel the goal after 30 seconds");
      this->client_->async_cancel_goal(goal_handle_);
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal is already completed or no valid goal handle exists.");
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperActionClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
