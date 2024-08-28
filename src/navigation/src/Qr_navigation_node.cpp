#include "navigation/Qr_navigation_node.hpp"
#include <chrono>
#include <functional>

using namespace std::chrono_literals;

QrNavigationNode::QrNavigationNode() : Node("Qr_navigation_node")
{
   
    detection_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "/qr_code_detection", 10,
        std::bind(&QrNavigationNode::detectionCallback, this, std::placeholders::_1));
    centroid_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/qr_code_centroid", 10,
        std::bind(&QrNavigationNode::centroidCallback, this, std::placeholders::_1));
    strip_subscription_ = this->create_subscription<std_msgs::msg::Int8>(
        "/lineDetectionData", 10,
        std::bind(&QrNavigationNode::stripCallback, this, std::placeholders::_1));

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

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

    timer_ = this->create_wall_timer(100ms, std::bind(&QrNavigationNode::timerCallback, this));
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

void QrNavigationNode::stripCallback(const std_msgs::msg::Int8::SharedPtr msg)
{
    is_at_strip_ = msg->data;

    // Convert the integer value to a binary string (assuming 5 sensors, use a 5-bit binary)
    std::bitset<5> binary_value(msg->data);

    // Assign each bit to a corresponding sensor variable
    sensor1 = binary_value[0]; 
    sensor2 = binary_value[1];  
    sensor3 = binary_value[2];  
    sensor4 = binary_value[3];  
    sensor5 = binary_value[4];  

    // Log the binary value and the individual sensor states
    RCLCPP_INFO(this->get_logger(), "Binary value: %s", binary_value.to_string().c_str());
    RCLCPP_INFO(this->get_logger(), "Sensor 1: %d, Sensor 2: %d, Sensor 3: %d, Sensor 4: %d, Sensor 5: %d", 
                sensor1, sensor2, sensor3, sensor4, sensor5);
}

void QrNavigationNode::timerCallback()
{
    geometry_msgs::msg::Twist cmd_vel_msg;

    if (!objectDetected) {
        cmd_vel_msg.linear.x = 0;
        cmd_vel_msg.angular.z = max_angular_speed_; // turn right

        RCLCPP_WARN(this->get_logger(), "No contours found");

    } else {

        RCLCPP_INFO(this->get_logger(), "Center X: %d", center_x);

        // Scale angular velocity based on how far the centroid is from the center
        double error = center_x - cx;
        double angular_z = angular_scaling_factor_ * error; // Adjust scaling factor as needed


        double linear_speed = 0;

        if(is_at_strip_ != 31){

            linear_speed = max_linear_speed_;

        }else{

            cmd_vel_msg.linear.x = 0.0;
            cmd_vel_msg.angular.z = 0.0;
            cmd_vel_publisher_->publish(cmd_vel_msg);
            rclcpp::shutdown();
            // HERE WE NEED TO STOP THE GRABBER ACTION AND TELL THE NODE THAT WE STOPPED
        }

        if (std::abs(error) < tolerance_) {
            cmd_vel_msg.linear.x = linear_speed; // Move forward
            cmd_vel_msg.angular.z = 0.0;
            // RCLCPP_INFO(this->get_logger(), "Centroid is near the center, moving forward");
        } else {
            cmd_vel_msg.linear.x = linear_speed;
            cmd_vel_msg.angular.z = angular_z;
            // RCLCPP_INFO(this->get_logger(), "Adjusting course: angular_z = %f", angular_z);
        }

    }
    cmd_vel_publisher_->publish(cmd_vel_msg);

}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<QrNavigationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
