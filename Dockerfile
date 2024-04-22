# Start from the base image
FROM nvcr.io/nvidia/pytorch:22.09-py3

# Install ROS2 Galactic
RUN apt-get update && \
    apt-get install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    apt-get update && \
    apt-get install -y ros-galactic-desktop && \
    rm -rf /var/lib/apt/lists/*

# Set the entrypoint to /bin/bash
ENTRYPOINT ["/bin/bash"]