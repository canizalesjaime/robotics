# Use ROS Noetic Desktop as the base image
FROM osrf/ros:noetic-desktop

SHELL [ "/bin/bash" , "-c" ]

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

RUN source /opt/ros/noetic/setup.bash && \
mkdir -p ~/catkin_ws/src/apollo_detection_systems && \
cd ~/catkin_ws && \
catkin_make && \
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

RUN echo "export ROS_MASTER_URI=http://192.168.1.5:11311" >> ~/.bashrc

# Set up environment
ENV DEBIAN_FRONTEND=noninteractive

# Update package lists and install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    python3-venv \
    #python3-tk \  # Install Tkinter
    git \
    wget \
    && apt-get clean

# Install a compatible version of NetworkX first (before PyTorch)
RUN pip3 install "networkx<3.0"

# Now install PyTorch with CUDA 11.7 support
RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu117

RUN apt-get update && apt-get install -y python3-tk
# Verify installation
#RUN python3 -c "import torch; import networkx; import tkinter; print('PyTorch:', torch.__version__, '| NetworkX:', networkx.__version__, '| Tkinter installed')"

# Set default command to bash
#CMD ["/bin/bash"]
