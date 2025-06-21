FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Add packages
RUN apt update && apt install -y \
    sudo \
    curl \
    gnupg2 \
    lsb-release \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    ros-humble-desktop \
    ros-humble-rviz2 \
    ros-dev-tools \
    && apt clean \
    && pip3 install pytransform3d

# Add ROS user
RUN useradd -m -s /bin/bash rosuser && \
    echo "rosuser ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Known issue handle
RUN mkdir -p /etc/ros/rosdep/sources.list.d && \
    rosdep init || echo "rosdep already initialized"

# non-root user
USER rosuser

# Aupdate rosdep
RUN rosdep update

# Workspace
WORKDIR /home/rosuser
RUN mkdir -p ros2_ws/src

# Source ROS 2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
