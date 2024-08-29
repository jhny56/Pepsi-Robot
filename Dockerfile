# Use the custom base image created earlier
FROM ros_galactic_with_pip

# Set environment variables
ENV ROS_WS=/ros2_ws
ENV ROS_DOMAIN_ID=0
ENV CMAKE_PREFIX_PATH=/opt/ros/galactic:$CMAKE_PREFIX_PATH

# Create a workspace directory
WORKDIR $ROS_WS/src

# Copy your ROS2 package into the workspace
COPY ./src/perception $ROS_WS/src/perception
COPY ./src/navigation $ROS_WS/src/navigation
COPY ./src/control $ROS_WS/src/control
COPY ./src/my_launch_package $ROS_WS/src/my_launch_package
COPY ./src/behavior_tree $ROS_WS/src/behavior_tree
COPY ./src/robot_hardware_interfaces $ROS_WS/src/robot_hardware_interfaces
# Copy the weights file into the Docker image
COPY weights.pt /ros2_ws/weights.pt 

# Go back to workspace root
WORKDIR $ROS_WS

# Build the workspace
RUN . /opt/ros/galactic/setup.sh && colcon build

# Navigate to the behavior tree build directory and clean it, then build it
WORKDIR $ROS_WS/src/behavior_tree
RUN . /opt/ros/galactic/setup.sh && mkdir -p build && cd build && rm -rf CMakeCache.txt CMakeFiles/ && cmake .. && make

# Source the workspace and setup entrypoint
WORKDIR $ROS_WS
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc
RUN echo "source /opt/ros/galactic/setup.bash" >> /root/.bashrc


# Set the default command to run when the container starts
CMD ["bash", "-c", "source /opt/ros/galactic/setup.bash && source /ros2_ws/install/setup.bash && cd /ros2_ws/src/behavior_tree/build && ./GROUPRMJ-ROBOTICS-COMPETITION"]
# CMD ["bash", "-c", "source /opt/ros/galactic/setup.bash && source /ros2_ws/install/setup.bash && ros2 run perception pepsican_detector"]
# CMD ["bash", "-c", "source /opt/ros/galactic/setup.bash && source /ros2_ws/install/setup.bash && ros2 run control gripper_action_client"]


