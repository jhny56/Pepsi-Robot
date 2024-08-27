#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "robot_hardware_interfaces/action/gripper_action.hpp"

class GripperActionClient : public rclcpp::Node
{
public:
  using GripperAction = robot_hardware_interfaces::action::GripperAction;
  using GoalHandleGripperAction = rclcpp_action::ClientGoalHandle<GripperAction>;

  GripperActionClient()
  : Node("gripper_action_client"), goal_active_(false)
  {
    this->client_ = rclcpp_action::create_client<GripperAction>(this, "gripper_action");

    this->send_goal();
  }

  void send_goal()
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

    // Set a timer to cancel the goal after 30 seconds
    this->cancel_timer_ = this->create_wall_timer(
      std::chrono::seconds(30),
      std::bind(&GripperActionClient::cancel_goal, this)
    );
  }

private:
  rclcpp_action::Client<GripperAction>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr cancel_timer_;
  std::shared_future<GoalHandleGripperAction::SharedPtr> goal_handle_future_;
  GoalHandleGripperAction::SharedPtr goal_handle_;
  bool goal_active_;

  void goal_response_callback(GoalHandleGripperAction::SharedPtr goal_handle)
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

  void feedback_callback(
    GoalHandleGripperAction::SharedPtr,
    const std::shared_ptr<const GripperAction::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Received feedback: %d", feedback->feedback);
  }

  void result_callback(const GoalHandleGripperAction::WrappedResult & result)
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

  void cancel_goal()
  {
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