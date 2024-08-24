# Use the custom base image created earlier
FROM ros_galactic_with_deps

# Set environment variables
ENV ROS_WS=/ros2_ws
ENV ROS_DOMAIN_ID=56

# Create a workspace directory
WORKDIR $ROS_WS/src

# Copy your ROS2 package into the workspace
COPY ./src/perception $ROS_WS/src/perception
COPY ./src/navigation $ROS_WS/src/navigation
COPY ./src/control $ROS_WS/src/control
COPY ./src/my_launch_package $ROS_WS/src/my_launch_package

# Go back to workspace root
WORKDIR $ROS_WS

# Build the workspace
RUN . /opt/ros/galactic/setup.sh && colcon build

# Source the workspace and setup entrypoint
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Set the default command to run when the container starts
CMD ["bash", "-c", "source /ros2_ws/install/setup.bash && ros2 launch my_launch_package launch_packages.py"]
