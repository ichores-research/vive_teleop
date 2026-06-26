FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /ros2_ws

SHELL ["/bin/bash", "-c"]

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=67
ENV PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-cv-bridge \
    ros-humble-geometry-msgs \
    ros-humble-vision-opencv \
    python3-colcon-common-extensions \
    python3-pip \
    python3-venv \
    python3-opencv \
    python3-dev \
    build-essential \
    cmake \
    git \
    curl \
    wget \
    nano \
    ffmpeg \
    pkg-config \
    libssl-dev \
    libffi-dev \
    libopencv-dev \
    libavdevice-dev \
    libavfilter-dev \
    libavformat-dev \
    libavcodec-dev \
    libswresample-dev \
    libswscale-dev \
    libavutil-dev \
    libopus-dev \
    libvpx-dev \
    libx264-dev \
    libx265-dev \
    libjpeg-dev \
    zlib1g-dev \
    && rm -rf /var/lib/apt/lists/*

COPY cyclonedds.xml /ros2_ws/cyclonedds.xml

COPY . /ros2_ws

COPY src/image_listener/requirements.txt /tmp/requirements.txt

# Pinned setuptools prevents the "--editable not recognized" error in Humble
RUN python3 -m pip install --upgrade pip "setuptools<58.0.0" wheel cython

RUN python3 -m pip install --no-cache-dir -r /tmp/requirements.txt

RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

CMD ["/bin/bash"]
