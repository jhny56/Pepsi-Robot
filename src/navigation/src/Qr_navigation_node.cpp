#include "navigation/Qr_navigation_node.hpp"

QrNavigationNode::QrNavigationNode() : Node("qr_navigation_node")
{
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10, std::bind(&QrNavigationNode::imageCallback, this, std::placeholders::_1));
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&QrNavigationNode::scanCallback, this, std::placeholders::_1));
    
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

void QrNavigationNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Get the index corresponding to the front of the robot
    // int mid_index = msg->ranges.size() / 2;
    front_distance_ = msg->ranges[0];
    
    RCLCPP_INFO(this->get_logger(), "Front distance: %f", front_distance_);
}

void QrNavigationNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Image received");

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        RCLCPP_INFO(this->get_logger(), "Image converted to OpenCV format");
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat hsv_image, mask;
    cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);
    RCLCPP_INFO(this->get_logger(), "Image converted to HSV");

    // Define the range for detecting the blue color
    cv::Scalar lower_blue(100, 150, 50);
    cv::Scalar upper_blue(140, 255, 255);
    cv::inRange(hsv_image, lower_blue, upper_blue, mask);
    RCLCPP_INFO(this->get_logger(), "Thresholding complete, searching for contours");

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    
    geometry_msgs::msg::Twist cmd_vel_msg;

    if (contours.empty()) {
        cmd_vel_msg.linear.x = 0;
        cmd_vel_msg.angular.z = 0.7; // turn right

        RCLCPP_WARN(this->get_logger(), "No contours found");
    } else {
        RCLCPP_INFO(this->get_logger(), "Found %lu contours", contours.size());

        // Calculate centroid and control logic
        cv::Moments M = cv::moments(contours[0]);
        int cx = int(M.m10 / M.m00);
        int cy = int(M.m01 / M.m00);

        RCLCPP_INFO(this->get_logger(), "Centroid of the largest contour: (%d, %d)", cx, cy);

        int tolerance = 100; // Adjust this value as needed (in pixels or as a percentage of image width)
        int center_x = mask.cols / 2;
        RCLCPP_INFO(this->get_logger(), "Center X: %d", center_x);

         // Add distance check from the laser scan data
        double stop_threshold = 0.3; // Minimum distance before stopping
        double slow_down_threshold = 2; // Distance where to start slowing down

        // Scale angular velocity based on how far the centroid is from the center
        double error = center_x - cx;
        double angular_z = +0.0005 * error; // Adjust scaling factor as needed

        double linear_speed = 0;
        RCLCPP_WARN(this->get_logger(), "Front Distance : %d",front_distance_);

        if(front_distance_ > slow_down_threshold){
            RCLCPP_WARN(this->get_logger(), "Object close! Slowing Down.");

            linear_speed = 0.5;

        }else if(front_distance_ > stop_threshold){
            RCLCPP_WARN(this->get_logger(), "Object close! Slowing Down.");

            // Gradually decrease speed as the robot approaches the object
            double speed_factor = (front_distance_ - stop_threshold) / (slow_down_threshold - stop_threshold);
            tolerance = 10;
            speed_factor = std::max(0.1, std::min(1.0, speed_factor));
            RCLCPP_WARN(this->get_logger(), "speed_factor: %d",speed_factor);

            linear_speed= 0.5 * speed_factor;
        }else{
            linear_speed= 0.0; // Stop if too close
            tolerance = 0;
            RCLCPP_WARN(this->get_logger(), "Object too close! Stopping.");
        }

        if (std::abs(error) < tolerance) {
            cmd_vel_msg.linear.x = linear_speed; // Move forward
            cmd_vel_msg.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Centroid is near the center, moving forward");
        } else {
            cmd_vel_msg.linear.x = linear_speed;
            cmd_vel_msg.angular.z = angular_z;
            RCLCPP_INFO(this->get_logger(), "Adjusting course: angular_z = %f", angular_z);
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
