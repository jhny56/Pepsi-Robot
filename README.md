ros2 pkg create --build-type ament_cmake --node-name initNode control --dependencies rclcpp rclpy

# To Build The Base Image (build once)
docker build -f Dockerfile.base -t ros_galactic_with_deps .

# To Build the custom image (takes less time)
docker build -t my_ros2_application .

# To Run the container
sudo docker run -it my_ros2_application 
or
sudo docker run -it my_ros2_application /bin/bash (to interact with the container)

