FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Update and install development tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    nano \
    vim \
    x11-apps \
    mesa-utils \
    build-essential \
    ros-jazzy-rviz2 \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*



RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /workspaces/homebot/ros2_ws/install/setup.bash" >> ~/.bashrc
# Initialize rosdep (ignore if already initialized)
#RUN rosdep init || true

# Create workspace
# RUN mkdir -p /root/ros2_ws/src

# WORKDIR /root/ros2_ws

# # Source ROS automatically
# RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
#     echo "source /root/ros2_ws/install/setup.bash 2>/dev/null || true" >> /root/.bashrc

# CMD ["bash"]