# Base image
FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=burger

# Install dependencies
RUN apt-get update && apt-get install -y \
    sudo x11-apps net-tools iputils-ping openssh-client \
    curl ros-jazzy-turtlebot3-* ros-dev-tools git \
    && apt-get clean

# Create user `ros` and give sudo
RUN userdel -r ubuntu && useradd -m -s /bin/bash ros && \
    echo "ros:ros" | chpasswd && \
    echo "root:root" | chpasswd && \
    usermod -aG sudo ros && \
    echo 'ros ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers.d/ros && \
    chmod 0440 /etc/sudoers.d/ros

USER ros
WORKDIR /home/ros

# ROS environment
RUN echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc && \
    echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
