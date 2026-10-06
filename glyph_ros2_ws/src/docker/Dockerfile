FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

# Install system and ROS packages required by the default robot launch path.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ffmpeg \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    mesa-utils \
    portaudio19-dev \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    ros-dev-tools \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Copy the workspace source tree and install Python runtime dependencies that
# are not fully declared in package.xml metadata.
COPY . /ros2_ws/src
COPY docker/requirements-docker.txt /tmp/requirements-docker.txt
RUN python3 -m pip install --no-cache-dir --upgrade pip && \
    python3 -m pip install --no-cache-dir -r /tmp/requirements-docker.txt

# Install ROS dependencies and build the workspace.
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build

# Source ROS and the workspace automatically in interactive shells.
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Runtime environment defaults.
ENV ROBOT_IP=""
ENV CONN_TYPE="webrtc"
ENV WEBRTC_SERVER_HOST="0.0.0.0"
ENV WEBRTC_SERVER_PORT="9991"

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "go2_robot_sdk", "robot.launch.py"]
