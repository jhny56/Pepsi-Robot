#include "navigation/can_navigation_node.hpp"

CanNavigationNode::CanNavigationNode() : Node("can_navigation_node")
{
     // Create callback groups
        // auto action_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        auto subscription_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // Subscriptions with ReentrantCallbackGroup
        rclcpp::SubscriptionOptions subscription_options;
        subscription_options.callback_group = subscription_callback_group;

        // Action server options
        rcl_action_server_options_t action_server_options = rcl_action_server_get_default_options();

        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&CanNavigationNode::scanCallback, this, std::placeholders::_1),
            subscription_options);

        detection_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "/pepsi_can_detection", 10,
            std::bind(&CanNavigationNode::detectionCallback, this, std::placeholders::_1),
            subscription_options);

        centroid_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/pepsi_can_centroid", 10,
            std::bind(&CanNavigationNode::centroidCallback, this, std::placeholders::_1),
            subscription_options);

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        is_at_can_publisher_ = this->create_publisher<std_msgs::msg::Bool>("is_at_can", 10);

        declare_parameter("tolerance", 100);
        declare_parameter("stop_threshold", 0.3f);
        declare_parameter("slow_down_threshold", 2.0f);
        declare_parameter("angular_scaling_factor", 0.0005f);
        declare_parameter("max_linear_speed", 0.5f);
        declare_parameter("max_angular_speed", 0.7f);

        // Action server creation
        action_server_ = rclcpp_action::create_server<GripperAction>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "can_navigation", 
            std::bind(&CanNavigationNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&CanNavigationNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&CanNavigationNode::execute, this, std::placeholders::_1),
            action_server_options, 
            subscription_callback_group  
            
        );

    int tolerance = get_parameter("tolerance").as_int();
    double stop_threshold = get_parameter("stop_threshold").as_double();
    double slow_down_threshold = get_parameter("slow_down_threshold").as_double();
    double angular_scaling_factor = get_parameter("angular_scaling_factor").as_double();
    double max_linear_speed = get_parameter("max_linear_speed").as_double();
    double max_angular_speed = get_parameter("max_angular_speed").as_double();

    if (this->has_parameter("max_angular_speed")) {
        RCLCPP_INFO(this->get_logger(), "max_angular_speed_ is set to: %f", max_angular_speed_);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to get max_angular_speed_ parameter");
    }
}

rclcpp_action::GoalResponse CanNavigationNode::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GripperAction::Goal> goal)
{
    // Convert the UUID to a string
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (const auto& byte : uuid) {
        ss << std::setw(2) << static_cast<int>(byte);
    }
    std::string uuid_str = ss.str();

    RCLCPP_INFO(this->get_logger(), "Received goal request with uuid %s", uuid_str.c_str());
    // Decide whether to accept the goal
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


rclcpp_action::CancelResponse CanNavigationNode::handle_cancel(
    const std::shared_ptr<GoalHandleCanNavigation> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    // Allow cancellation
    return rclcpp_action::CancelResponse::ACCEPT;
}

void CanNavigationNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    front_distance_ = msg->ranges[0];
    RCLCPP_ERROR(this->get_logger(), "scanCallback");

}

void CanNavigationNode::detectionCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    objectDetected = msg->data;
    RCLCPP_ERROR(this->get_logger(), "detectionCallback");

}

void CanNavigationNode::centroidCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    cx = msg->x;
    center_x = msg->z;
    RCLCPP_ERROR(this->get_logger(), "centroidCallback");

}


void CanNavigationNode::execute(const std::shared_ptr<GoalHandleCanNavigation> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<GripperAction::Result>();

    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok())
    {
        
        if (!objectDetected)
        {
            geometry_msgs::msg::Twist cmd_vel_msg;
            cmd_vel_msg.linear.x = 0;
            cmd_vel_msg.angular.z = max_angular_speed_; // turn right

            RCLCPP_WARN(this->get_logger(), "No contours found");
            cmd_vel_publisher_->publish(cmd_vel_msg);
        }
        else
        {
            geometry_msgs::msg::Twist cmd_vel_msg;
            RCLCPP_INFO(this->get_logger(), "Center X: %f", center_x);

            double error = center_x - cx;
            double angular_z = angular_scaling_factor_ * error; // Adjust scaling factor as needed

            double linear_speed = 0;

            if (front_distance_ > slow_down_threshold_)
            {
                linear_speed = max_linear_speed_;
            }
            else if (front_distance_ > stop_threshold_)
            {
                double speed_factor = (front_distance_ - stop_threshold_) / (slow_down_threshold_ - stop_threshold_);
                tolerance_ = 10;
                speed_factor = std::max(0.1, std::min(1.0, speed_factor));
                RCLCPP_WARN(this->get_logger(), "speed_factor: %f", speed_factor);

                linear_speed = max_linear_speed_ * speed_factor;
            }
            else
            {
                cmd_vel_msg.linear.x = 0.0; // Stop if too close
                cmd_vel_msg.angular.z = 0.0;
                tolerance_ = 0;

                // Publish true to is_at_can topic
                std_msgs::msg::Bool is_at_can_msg;
                is_at_can_msg.data = true;
                is_at_can_publisher_->publish(is_at_can_msg);

                // Set the result before succeeding
                result->result = true; // Assuming the action was successful
                RCLCPP_WARN(this->get_logger(), "PUBLISHING GOAL SUCCEED");

                goal_handle->succeed(result);
                cmd_vel_publisher_->publish(cmd_vel_msg);
                return;
            }

            
            if (std::abs(error) < tolerance_)
            {
                cmd_vel_msg.linear.x = linear_speed; // Move forward
                cmd_vel_msg.angular.z = 0.0;
            }
            else
            {
                cmd_vel_msg.linear.x = linear_speed;
                cmd_vel_msg.angular.z = angular_z;
            }
            cmd_vel_publisher_->publish(cmd_vel_msg);

            // Provide feedback
            auto feedback = std::make_shared<GripperAction::Feedback>();
            feedback->feedback = static_cast<int32_t>(front_distance_);
            goal_handle->publish_feedback(feedback);
        }

        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CanNavigationNode>();

    // Set the maximum number of threads to 4 (or any other number you prefer)
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 8);
    executor.add_node(node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
