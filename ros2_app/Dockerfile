FROM ros:foxy

WORKDIR /ros2_ws

# Install dependencies
RUN apt update && apt install -y \
    ros-foxy-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions \
    curl \
    ros-foxy-cv-bridge \
    ros-foxy-vision-opencv \
    python3-pip \
    python3-opencv \
    libavdevice-dev \
    libavfilter-dev \
    libavformat-dev \
    libavcodec-dev \
    libswresample-dev \
    libswscale-dev \
    libavutil-dev \
    libopus-dev \
    libvpx-dev \
    pkg-config \
    libssl-dev \
    && rm -rf /var/lib/apt/lists/*

COPY cyclonedds.xml /ros2_ws/cyclonedds.xml
COPY . /ros2_ws


COPY src/image_listener/requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build --symlink-install"

# Add wait script
COPY <<EOF /wait-for-bridge.sh
#!/bin/bash
set -e

source /opt/ros/foxy/setup.bash
source /ros2_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///ros2_ws/cyclonedds.xml

echo "Waiting for ros1_bridge topics..."
sleep 10
TOPIC="/xtion/rgb/image_raw"

for i in {1..60}; do
    echo "Attempt $i/60"

    ros2 topic list

    if ros2 topic list 2>/dev/null | grep -q .; then
        echo "ROS2 topics detected"
        exit 0
    fi

    sleep 3
done

echo "Timeout waiting for topic $TOPIC"
exit 1
EOF

RUN chmod +x /wait-for-bridge.sh

CMD ["/bin/bash"]