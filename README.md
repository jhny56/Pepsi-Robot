ros2 pkg create --build-type ament_cmake --node-name initNode control --dependencies rclcpp rclpy

sudo docker build -t my_ros2_project .
sudo docker run -it my_ros2_project
