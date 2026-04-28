FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# Install core tooling and debug utilities
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    git \
    build-essential \
    gdb \
    valgrind \
    strace \
    tzdata \
    && rm -rf /var/lib/apt/lists/*

# Add ROS 1 and ROS 2 repositories
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros1.list

# Install ROS 1 Noetic, ROS 2 Foxy, and build tools
RUN apt-get update && apt-get install -y \
    ros-noetic-ros-base \
    ros-foxy-ros-base \
    ros-foxy-demo-nodes-cpp \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    ros-foxy-rmw-cyclonedds-cpp \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# Python OpenVR bindings for vive_head_pose scripts
RUN pip3 install --no-cache-dir openvr

WORKDIR /bridge_ws

# Pull the specific branch matching our ROS 2 distro
RUN git clone -b foxy https://github.com/ros2/ros1_bridge.git src/ros1_bridge

# Resolve ROS dependencies
RUN rosdep init && rosdep update
RUN /bin/bash -c "set -e; \
    source /opt/ros/noetic/setup.bash; \
    source /opt/ros/foxy/setup.bash; \
    apt-get update; \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro foxy" \
    && rm -rf /var/lib/apt/lists/*

# Build the bridge with debug flags.
# Both ROS distros must be sourced in the same shell prior to building.
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
                  source /opt/ros/foxy/setup.bash && \
                  colcon build \
                    --symlink-install \
                    --cmake-args \
                    -DCMAKE_BUILD_TYPE=Debug \
                    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"

# ROS 2 Foxy: Vive HMD pose publisher
COPY ros2_ws /ros2_vive_ws
WORKDIR /ros2_vive_ws
RUN /bin/bash -c "set -e; \
    source /opt/ros/foxy/setup.bash; \
    apt-get update; \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro foxy" \
    && rm -rf /var/lib/apt/lists/*
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && \
    cd /ros2_vive_ws && \
    colcon build --symlink-install --packages-select vive_head_pose"

# ROS 1 Noetic: TIAGO head mapper (optional; often run on a ROS 1 machine with ROS_MASTER_URI)
COPY ros1_ws /catkin_ws
WORKDIR /catkin_ws
RUN /bin/bash -c "set -e; \
    source /opt/ros/noetic/setup.bash; \
    apt-get update; \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro noetic" \
    && rm -rf /var/lib/apt/lists/*
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /catkin_ws && catkin_make"

# Shell setup for interactive debugging in container
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc \
    && echo 'source /opt/ros/foxy/setup.bash' >> /root/.bashrc \
    && echo 'source /bridge_ws/install/setup.bash' >> /root/.bashrc \
    && echo 'source /ros2_vive_ws/install/setup.bash' >> /root/.bashrc \
    && echo 'source /catkin_ws/devel/setup.bash' >> /root/.bashrc

# TODO: On next big change move it up, for now its here so docker caches previous big steps
RUN apt-get update && apt-get install -y iproute2 && rm -rf /var/lib/apt/lists/*

COPY bridge/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

WORKDIR /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]
