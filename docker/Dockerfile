# docker/Dockerfile
FROM dustynv/ros:humble-desktop-l4t-r36.2.0

ENV DEBIAN_FRONTEND=noninteractive

# 1. Install System Dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget zstd build-essential cmake git python3-pip \
    libusb-1.0-0-dev python3-pyroute2 lsb-release gnupg2 nano \
    libqt5xml5 libx11-dev \
    && rm -rf /var/lib/apt/lists/*

# 2. Create 'ubuntu' User
RUN useradd -m -s /bin/bash ubuntu && \
    echo "ubuntu:ubuntu" | chpasswd && \
    usermod -aG sudo,video,dialout ubuntu

# 3. Install ZED SDK 5.2 (JetPack 6 / L4T 36.3)
ARG ZED_SDK_URL="https://download.stereolabs.com/zedsdk/5.2/l4t36.3/jetsons"
RUN wget -q --no-check-certificate -O ZED_SDK_Linux.run ${ZED_SDK_URL} && \
    chmod +x ZED_SDK_Linux.run && \
    ./ZED_SDK_Linux.run -- silent skip_tools && \
    rm ZED_SDK_Linux.run

# 4. Install Python Libs
RUN pip3 install --no-cache-dir ultralytics pyserial cython numpy

# 5. Setup ROS 2 Workspace & ZED Wrapper
USER ubuntu
WORKDIR /home/ubuntu/ros2_ws/src

RUN git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git && \
    git clone https://github.com/stereolabs/zed-ros2-interfaces.git

# 6. Build the Base Workspace
WORKDIR /home/ubuntu/ros2_ws
SHELL ["/bin/bash", "-c"] 
RUN source /opt/ros/humble/setup.bash && \
    sudo apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 7. Source ROS Automatically
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /home/ubuntu/ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]