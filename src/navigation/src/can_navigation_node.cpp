#include "navigation/can_navigation_node.hpp"

CanNavigationNode::CanNavigationNode() : Node("can_navigation_node")
{
    
    scan_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
        "/distanceToObstacle", 10, std::bind(&CanNavigationNode::scanCallback, this, std::placeholders::_1));
    detection_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "/pepsi_can_detection", 10,
        std::bind(&CanNavigationNode::detectionCallback, this, std::placeholders::_1));
    centroid_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/pepsi_can_centroid", 10,
        std::bind(&CanNavigationNode::centroidCallback, this, std::placeholders::_1));

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    is_at_can_publisher_ = this->create_publisher<std_msgs::msg::Bool>("is_at_can", 10);

    declare_parameter("tolerance", 100);
    declare_parameter("stop_threshold", 0.3f);
    declare_parameter("slow_down_threshold", 2.0f);
    declare_parameter("angular_scaling_factor", 0.0005f);
    declare_parameter("max_linear_speed", 0.5f);
    declare_parameter("max_angular_speed", 0.7f);

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

void CanNavigationNode::scanCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    front_distance_ = msg->data; //TODO
}

void CanNavigationNode::detectionCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    objectDetected = msg->data;
}

void CanNavigationNode::centroidCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    cx = msg->x;
    center_x = msg->z;

    geometry_msgs::msg::Twist cmd_vel_msg;

    if (!objectDetected) {
        cmd_vel_msg.linear.x = 0;
        cmd_vel_msg.angular.z = max_angular_speed_; // turn right

        RCLCPP_WARN(this->get_logger(), "No contours found");
    } else {
        RCLCPP_INFO(this->get_logger(), "Center X: %d", center_x);

        double error = center_x - cx;
        double angular_z = angular_scaling_factor_ * error; // Adjust scaling factor as needed

        double linear_speed = 0;

        if (front_distance_ > slow_down_threshold_) {
            linear_speed = max_linear_speed_;
        } else if (front_distance_ > stop_threshold_) {
            double speed_factor = (front_distance_ - stop_threshold_) / (slow_down_threshold_ - stop_threshold_);
            tolerance_ = 10;
            speed_factor = std::max(0.1, std::min(1.0, speed_factor));
            RCLCPP_WARN(this->get_logger(), "speed_factor: %f", speed_factor);

            linear_speed = max_linear_speed_ * speed_factor;
        } else {
            linear_speed = 0.0; // Stop if too close
            tolerance_ = 0;

            // Publish true to is_at_can topic
            std_msgs::msg::Bool is_at_can_msg;
            is_at_can_msg.data = true;
            is_at_can_publisher_->publish(is_at_can_msg);
            rclcpp::shutdown();
        }

        if (std::abs(error) < tolerance_) {
            cmd_vel_msg.linear.x = linear_speed; // Move forward
            cmd_vel_msg.angular.z = 0.0;
        } else {
            cmd_vel_msg.linear.x = linear_speed;
            cmd_vel_msg.angular.z = angular_z;
        }
    }
    cmd_vel_publisher_->publish(cmd_vel_msg);
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanNavigationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
