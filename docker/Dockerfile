# 1. Base Image (NVIDIA optimized ROS 2)
FROM docker.io/dustynv/ros:humble-desktop-l4t-r36.2.0

# Prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# 2. Fix ROS GPG Key and Add Official ROS 2 Repo
RUN apt-get update || true && apt-get install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list

# 3. Essential System Dependencies (Surgical Strike)
# Removed the rosidl packages to prevent them from colliding with NVIDIA
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-nmea-msgs \
    ros-humble-geographic-msgs \
    ros-humble-robot-localization \
    ros-humble-diagnostic-updater \
    ros-humble-xacro \
    ros-humble-image-transport \
    ros-humble-point-cloud-transport \
    ros-humble-backward-ros \
    ros-humble-tf2-sensor-msgs \
    wget zstd git python3-pip \
    libusb-1.0-0-dev vim \
    libqt5xml5 libx11-dev \
    && rm -rf /var/lib/apt/lists/*

# 4. ZED SDK (L4T 36.3 version for JetPack 6)
RUN wget -q --no-check-certificate -O ZED_SDK_Linux.run https://download.stereolabs.com/zedsdk/5.2/l4t36.3/jetsons && \
    chmod +x ZED_SDK_Linux.run && \
    ./ZED_SDK_Linux.run -- silent && \
    rm ZED_SDK_Linux.run

# 5. Python AI & Hardware Libs
RUN pip3 install --no-cache-dir --ignore-installed \
    ultralytics \
    pyserial \
    cython \
    "numpy<2.0" \
    sympy \
    typing-extensions

# 6. Workspace Setup
WORKDIR /ros2_ws/src
RUN git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git && \
    git clone https://github.com/stereolabs/zed-ros2-interfaces && \
    git clone -b humble https://github.com/ros-perception/vision_opencv.git

# 7. The Build (The Surgical Metapackage Fix)
WORKDIR /ros2_ws
SHELL ["/bin/bash", "-c"]
# 1. Source the paths
# 2. Delete the broken hardware debugger
# 3. Surgically remove zed_debug from the zed_ros2 metapackage XML
# 4. Build ONLY zed_msgs first to guarantee the headers exist
# 5. Source the new msgs, then build the rest
RUN source /opt/ros/humble/setup.bash && \
    source /opt/ros/humble/install/setup.bash && \
    rm -rf src/zed-ros2-wrapper/zed_debug && \
    sed -i '/zed_debug/d' src/zed-ros2-wrapper/zed_ros2/package.xml && \
    colcon build --packages-select zed_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    source install/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

# 8. Environment Persistence
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /opt/ros/humble/install/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# 9. Custom Entrypoint
RUN echo "#!/bin/bash" > /usv_entrypoint.sh && \
    echo "set -e" >> /usv_entrypoint.sh && \
    echo "source /opt/ros/humble/setup.bash" >> /usv_entrypoint.sh && \
    echo "source /opt/ros/humble/install/setup.bash" >> /usv_entrypoint.sh && \
    echo "source /ros2_ws/install/setup.bash" >> /usv_entrypoint.sh && \
    echo 'exec "$@"' >> /usv_entrypoint.sh && \
    chmod +x /usv_entrypoint.sh

ENTRYPOINT ["/usv_entrypoint.sh"]
CMD ["bash"]