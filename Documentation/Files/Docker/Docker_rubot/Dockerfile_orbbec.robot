FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    pkg-config \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    wget \
    unzip \
    git \
    iputils-ping \
    libusb-1.0-0-dev \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    nlohmann-json3-dev \
    libdw-dev \
    libssl-dev \
    libgl1 \
    mesa-utils \
    ros-humble-xacro \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    ros-humble-image-geometry \
    ros-humble-image-publisher \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-compressed-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-diagnostic-updater \
    ros-humble-diagnostic-msgs \
    ros-humble-statistics-msgs \
    ros-humble-backward-ros \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-tf2-ros \
    ros-humble-v4l2-camera \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-demo-nodes-cpp \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rosbridge-server \
    ros-humble-rviz2 \
    ros-humble-rqt-graph \
    ros-humble-rqt-plot \
    ros-humble-nav2-bringup \
    ros-humble-nav2-simple-commander \
    ros-humble-tf-transformations \
    ros-humble-cartographer-ros \
    x11-apps \
    libgl1-mesa-glx \
    libqt5x11extras5 \
    libxkbcommon-x11-0 \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir pyserial

# --------------------------------------------------
# Orbbec workspace
# --------------------------------------------------
WORKDIR /root
RUN mkdir -p /root/orbbec_ws/src

WORKDIR /root/orbbec_ws/src
RUN git clone https://github.com/orbbec/OrbbecSDK_ROS2.git && \
    cd OrbbecSDK_ROS2 && \
    git checkout v2-main

WORKDIR /root/orbbec_ws
RUN source /opt/ros/humble/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install

# --------------------------------------------------
# Robot workspace
# --------------------------------------------------
WORKDIR /root

RUN source /opt/ros/humble/setup.bash && \
    source /root/orbbec_ws/install/setup.bash

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/orbbec_ws/install/setup.bash" >> /root/.bashrc

COPY --chmod=755 entrypoint.robot.sh /entrypoint.robot.sh
COPY --chmod=755 bringup.sh /bringup.sh

ENTRYPOINT ["/entrypoint.robot.sh"]
CMD ["/bringup.sh"]