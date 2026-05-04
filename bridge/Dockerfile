FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# Install core tooling and debug utilities
RUN echo 'Acquire::Retries "3";' > /etc/apt/apt.conf.d/80-retries \
    && apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg2 \
    lsb-release \
    git \
    build-essential \
    gdb \
    valgrind \
    strace \
    tzdata \
    iproute2 \
    && rm -rf /var/lib/apt/lists/*

# Add ROS 1 and ROS 2 repositories
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros1.list

# Following steps are split into smaller steps to leverage docker cache
# lab internet is so bad it cant handle downloading the whole ros1 and ros2 distributions
# at once without breaking, so we download them one by one over multiple runs

# Install ROS 1 Noetic Base
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 Foxy Base
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-ros-base \
    && rm -rf /var/lib/apt/lists/*

# Install Build Tools and Middleware
RUN apt-get update && apt-get install -y --no-install-recommends \
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

CMD ["/bin/bash"]
