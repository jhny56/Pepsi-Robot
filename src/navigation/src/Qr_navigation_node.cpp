#include "navigation/Qr_navigation_node.hpp"

QrNavigationNode::QrNavigationNode() : Node("qr_navigation_node")
{
    auto subscription_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    // Subscriptions with ReentrantCallbackGroup
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = subscription_callback_group;

    rcl_action_server_options_t action_server_options = rcl_action_server_get_default_options();

    // Subscriptions
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&QrNavigationNode::scanCallback, this, std::placeholders::_1),subscription_options);
    detection_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "/qr_code_detection", 10, std::bind(&QrNavigationNode::detectionCallback, this, std::placeholders::_1),subscription_options);
    centroid_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/qr_code_centroid", 10, std::bind(&QrNavigationNode::centroidCallback, this, std::placeholders::_1),subscription_options);
    strip_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "/is_at_strip", 10, std::bind(&QrNavigationNode::stripCallback, this, std::placeholders::_1),subscription_options);

    // Publishers
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    is_at_qr_publisher_ = this->create_publisher<std_msgs::msg::Bool>("is_at_qr", 10);

    // Declare and retrieve parameters
    declare_parameter("tolerance", 100);
    declare_parameter("stop_threshold", 0.3f);
    declare_parameter("slow_down_threshold", 2.0f);
    declare_parameter("angular_scaling_factor", 0.0005f);
    declare_parameter("max_linear_speed", 0.5f);
    declare_parameter("max_angular_speed", 0.7f);

    // Action server creation
    action_server_ = rclcpp_action::create_server<GripperAction>(
        this,
        "qr_navigation",
        std::bind(&QrNavigationNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&QrNavigationNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&QrNavigationNode::execute, this, std::placeholders::_1),
        action_server_options,
        subscription_callback_group
    );
}

rclcpp_action::GoalResponse QrNavigationNode::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GripperAction::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse QrNavigationNode::handle_cancel(
    const std::shared_ptr<GoalHandleQrNavigation> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void QrNavigationNode::execute(const std::shared_ptr<GoalHandleQrNavigation> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing QR navigation goal");

    auto result = std::make_shared<GripperAction::Result>();
    rclcpp::Rate loop_rate(10);

    while (rclcpp::ok())
    {
        geometry_msgs::msg::Twist cmd_vel_msg;

        if (!objectDetected) {
            cmd_vel_msg.linear.x = 0;
            cmd_vel_msg.angular.z = max_angular_speed_;  // turn right
            RCLCPP_WARN(this->get_logger(), "No QR code detected");
        } else {
            double error = center_x - cx;
            double angular_z = angular_scaling_factor_ * error;
            double linear_speed = (is_at_strip_) ? 0.0 : max_linear_speed_;

            if (std::abs(error) < tolerance_) {
                cmd_vel_msg.linear.x = linear_speed;
                cmd_vel_msg.angular.z = 0.0;
            } else {
                cmd_vel_msg.linear.x = linear_speed;
                cmd_vel_msg.angular.z = angular_z;
            }

            if (is_at_strip_) {
                RCLCPP_WARN(this->get_logger(), "At the strip, stopping");
                std_msgs::msg::Bool is_at_qr_msg;
                is_at_qr_msg.data = true;
                is_at_qr_publisher_->publish(is_at_qr_msg);

                result->result = true;
                goal_handle->succeed(result);
                return;
            }
        }

        cmd_vel_publisher_->publish(cmd_vel_msg);
        loop_rate.sleep();
    }
}

// Callbacks for subscriptions
void QrNavigationNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    front_distance_ = msg->ranges[0];
}

void QrNavigationNode::detectionCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    objectDetected = msg->data;
}

void QrNavigationNode::centroidCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    cx = msg->x;
    center_x = msg->z;
}

void QrNavigationNode::stripCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    is_at_strip_ = msg->data;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<QrNavigationNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
