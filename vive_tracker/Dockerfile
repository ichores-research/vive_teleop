FROM ros:foxy-ros-base

# Prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install critical dependencies for libsurvive and math libraries
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    freeglut3-dev \
    libatlas-base-dev \
    liblapacke-dev \
    libopenblas-dev \
    libpcap-dev \
    libusb-1.0-0-dev \
    libx11-dev \
    zlib1g-dev \
    && rm -rf /var/lib/apt/lists/*

# Set up the ROS 2 workspace
WORKDIR /ros2_ws
RUN mkdir -p src && cd src && \
    git clone https://github.com/asymingt/libsurvive_ros2.git

# Build the workspace (this wrapper builds libsurvive internally)
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build --packages-select libsurvive_ros2"

# Copy and setup entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

# Use the correct package name and launch file
CMD ["ros2", "launch", "libsurvive_ros2", "libsurvive_ros2.launch.py"]
