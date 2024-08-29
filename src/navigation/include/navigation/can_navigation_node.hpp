#ifndef CAN_NAVIGATION_NODE_HPP_
#define CAN_NAVIGATION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float32.hpp"

class CanNavigationNode : public rclcpp::Node
{
public:
    CanNavigationNode();

private:
    void scanCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void detectionCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void centroidCallback(const geometry_msgs::msg::Point::SharedPtr msg);
    void timerCallback();

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr detection_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr scan_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr centroid_subscription_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_at_can_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    float front_distance_;
    int tolerance_ = 100;
    double stop_threshold_ = 6;
    double slow_down_threshold_ = 15;
    double angular_scaling_factor_ = 0.0005;
    double max_linear_speed_ = 0.3;
    double max_angular_speed_ = 0.3;
    bool objectDetected = false;

    float cx = 0;
    float center_x = 0;
};

#endif  // CAN_NAVIGATION_NODE_HPP_
