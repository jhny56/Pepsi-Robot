# Use the official ROS Galactic base image
FROM ros:galactic

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS=/ros2_ws

# Install necessary packages
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    build-essential \
    git \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace directory
WORKDIR $ROS_WS/src

# Copy your ROS2 package into the workspace
COPY ./src/perception $ROS_WS/src/perception
COPY ./src/navigation $ROS_WS/src/navigation
COPY ./src/control $ROS_WS/src/control

COPY ./src/my_launch_package $ROS_WS/src/my_launch_package

# Go back to workspace root
WORKDIR $ROS_WS

# Install dependencies (if you have any dependencies in your package.xml)
# RUN apt-get update && rosdep update \
#     && rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN . /opt/ros/galactic/setup.sh && colcon build

# Source the workspace and setup entrypoint
RUN echo "source /opt/ros/galactic/setup.sh" >> /root/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc
ENTRYPOINT ["/bin/bash", "-c"]

# Command to run the container
CMD ["source /ros2_ws/install/setup.bash && ros2 launch my_launch_package launch_packages.py"]
