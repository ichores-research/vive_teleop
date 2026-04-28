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
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-foxy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /bridge_ws

# Pull the specific branch matching our ROS 2 distro
RUN git clone -b foxy https://github.com/ros2/ros1_bridge.git src/ros1_bridge

# Resolve ROS dependencies
RUN rosdep init && rosdep update
RUN apt-get update && rosdep install --from-paths src --ignore-src -r -y \
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

# Set up standard bash entry
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc \
    && echo 'source /opt/ros/foxy/setup.bash' >> /root/.bashrc \
    && echo 'source /bridge_ws/install/setup.bash' >> /root/.bashrc

# TODO: On next big change move it up, for now its here so docker caches previous big steps
RUN apt-get update && apt-get install -y iproute2 && rm -rf /var/lib/apt/lists/*

CMD ["/bin/bash"]
