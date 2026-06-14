FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=67
ENV PYTHONUNBUFFERED=1

SHELL ["/bin/bash", "-c"]

WORKDIR /moveit_ws

RUN apt-get update && apt-get install -y \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-moveit \
    ros-humble-moveit-servo \
    ros-humble-tiago-moveit-config \
    python3-colcon-common-extensions \
    python3-setuptools \
    build-essential \
    cmake \
    git \
    curl \
    nano \
    && rm -rf /var/lib/apt/lists/*

COPY src /moveit_ws/src

RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "vive_moveit_server", "vive_moveit_server.launch.py"]
