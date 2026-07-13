FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=67

SHELL ["/bin/bash", "-c"]
WORKDIR /recorder_ws

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-control-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-tf2-msgs \
    ros-humble-trajectory-msgs \
    && rm -rf /var/lib/apt/lists/*

COPY src /recorder_ws/src

RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF && \
    ros2 bag record --help >/dev/null && \
    ros2 pkg prefix rosbag2_storage_mcap >/dev/null

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "run", "vive_dataset_recorder", "vive_dataset_recorder"]
