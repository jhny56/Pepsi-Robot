#include "navigation/Qr_navigation_node.hpp"

QrNavigationNode::QrNavigationNode() : Node("Qr_navigation_node")
{
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10, std::bind(&QrNavigationNode::imageCallback, this, std::placeholders::_1));
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&QrNavigationNode::scanCallback, this, std::placeholders::_1));
    detection_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "/qr_code_detection", 10,
        std::bind(&QrNavigationNode::detectionCallback, this, std::placeholders::_1));
    centroid_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/qr_code_centroid", 10,
        std::bind(&QrNavigationNode::centroidCallback, this, std::placeholders::_1));
    strip_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "/is_at_strip", 10,
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
}

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

void QrNavigationNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Image received");

    // cv_bridge::CvImagePtr cv_ptr;
    // try
    // {
    //     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    //     // RCLCPP_INFO(this->get_logger(), "Image converted to OpenCV format");
    // }
    // catch (cv_bridge::Exception& e)
    // {
    //     RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    //     return;
    // }

    // cv::Mat hsv_image, mask;
    // cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);
    // // RCLCPP_INFO(this->get_logger(), "Image converted to HSV");

    // // Define the range for detecting the blue color
    // cv::Scalar lower_blue(100, 150, 50);
    // cv::Scalar upper_blue(140, 255, 255);
    // cv::inRange(hsv_image, lower_blue, upper_blue, mask);
    // // RCLCPP_INFO(this->get_logger(), "Thresholding complete, searching for contours");

    // // Find contours
    // std::vector<std::vector<cv::Point>> contours;
    // cv::findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    
    geometry_msgs::msg::Twist cmd_vel_msg;

    if (!objectDetected) {
        cmd_vel_msg.linear.x = 0;
        cmd_vel_msg.angular.z = max_angular_speed_; // turn right

        RCLCPP_WARN(this->get_logger(), "No contours found");
        // RCLCPP_INFO(this->get_logger(), "max_angular_speed_: %f", max_angular_speed_);

    } else {
        // RCLCPP_INFO(this->get_logger(), "Found %lu contours", contours.size());

        // // Calculate centroid and control logic
        // cv::Moments M = cv::moments(contours[0]);
        // int cx = int(M.m10 / M.m00);
        // int cy = int(M.m01 / M.m00);

        // RCLCPP_INFO(this->get_logger(), "Centroid of the largest contour: (%d, %d)", cx, cy);

        // int center_x = mask.cols / 2;
        RCLCPP_INFO(this->get_logger(), "Center X: %d", center_x);

         // Add distance check from the laser scan data

        // Scale angular velocity based on how far the centroid is from the center
        double error = center_x - cx;
        double angular_z = angular_scaling_factor_ * error; // Adjust scaling factor as needed


        double linear_speed = 0;
        // RCLCPP_WARN(this->get_logger(), "Front Distance : %f",front_distance_);

        if(!is_at_strip_){
            // RCLCPP_WARN(this->get_logger(), "Object close! Slowing Down.");

            linear_speed = max_linear_speed_;

        }else{
            linear_speed= 0.0; // Stop if too close
            tolerance_ = 0;


            // rclcpp::shutdown();
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
