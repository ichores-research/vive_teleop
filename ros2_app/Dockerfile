FROM ros:foxy

WORKDIR /ros2_ws

COPY . /ros2_ws

RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build"

CMD ["/bin/bash"]
