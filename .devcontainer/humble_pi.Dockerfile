#FROM osrf/ros:jazzy-desktop
#FROM jaimec21/jazzy_pi:latest
FROM ros:jazzy


# ENV DEBIAN_FRONTEND=noninteractive

# Update and install development tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    x11-apps \
    mesa-utils \
    build-essential \
    ros-jazzy-rviz2 \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    git \
    neovim \
    tmux \
    curl \
    ripgrep \
    fd-find \
    #unzip \ # these are for markdown and pdf viewers but didnt work for some reason
    #pandoc \
    #poppler-utils \
    #glow \
    #zathura \
    && rm -rf /var/lib/apt/lists/*


RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /workspaces/homebot/ros2_ws/install/setup.bash" >> ~/.bashrc

# # for pi 
# RUN apt update && apt upgrade -y
# RUN apt install -y \
#  python3-lgpio \
#  gpiod \
#  libgpiod-dev \
#  python3-libgpiod \
#  ros-jazzy-cv-bridge \
#  python3-opencv \
#  python3-pip \
#  python3-smbus \
#  i2c-tools \
# #  libcap-dev\ # camera stuff below
# #  libcamera-dev \
# #  libcamera-tools  \
# #  python3-libcamera \
# #  python3-pyqt6

# RUN pip3 install mpu6050-raspberrypi adafruit-blinka adafruit-circuitpython-pca9685 --break-system-packages\
#RUN sudo apt install ros-jazzy-tf-transformations
#RUN pip3 install fastapi uvicorn ultralytics opencv-python --break-system-packages


#RUN pip3 install picamera2 --break-system-packages


# still gotts do in image
#RUN pip3 install ultralytics opencv-python --break-system-packages
