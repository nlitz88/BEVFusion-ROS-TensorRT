# Start from the base image
FROM nvcr.io/nvidia/pytorch:22.09-py3

# TAKEN FROM ISAAC ROS COMMON:
# https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/c3a004d92da04c1ebeebcb094e5828458c19021f/docker/Dockerfile.ros2_humble

# disable terminal interaction for apt
ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"]

# Env setup
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV ROS_PYTHON_VERSION=3
ENV ROS_DISTRO=galactic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Avoid setup.py and easy_install deprecation warnings caused by colcon and setuptools
# https://github.com/colcon/colcon-core/issues/454
ENV PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources
RUN echo "Warning: Using the PYTHONWARNINGS environment variable to silence setup.py and easy_install deprecation warnings caused by colcon"

# Add ROS 2 apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2-testing/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Basics
RUN apt-get update && apt-get install -y \
        curl \
        gnupg \
        lsb-release \
&& rm -rf /var/lib/apt/lists/* \
&& apt-get clean

# ROS fundamentals
RUN apt-get update && apt-get install -y \
        build-essential \
        git \
        libyaml-cpp-dev=0.7.0+dfsg-8backport \
        python3-colcon-common-extensions \
        python3-pip \
        python3-pybind11 \
        python3-pytest-cov \
        python3-rosdep \
        python3-rosinstall-generator \
        python3-vcstool \
        wget \
&& rm -rf /var/lib/apt/lists/* \
&& apt-get clean

# ROS Python fundamentals
RUN python3 -m pip install -U \
        flake8-blind-except \
        flake8-builtins \
        flake8-class-newline \
        flake8-comprehensions \
        flake8-deprecated \
        flake8-docstrings \
        flake8-import-order \
        flake8-quotes \
        pytest-repeat \
        pytest-rerunfailures \
        pytest \
        setuptools==65.7.0

# Basics
RUN apt-get update && apt-get install -y \
        curl \
        gnupg \
        locales \
        lsb-release \
        python3-bloom \
        libxtensor-dev \
&& rm -rf /var/lib/apt/lists/* \
&& apt-get clean

# Install ROS 2 Humble
RUN apt-get update && apt-get install -y ros-humble-ros-base \
    ros-humble-angles \
    ros-humble-apriltag \
    ros-humble-behaviortree-cpp-v3 \
    ros-humble-bondcpp \
    ros-humble-camera-calibration-parsers \
    ros-humble-camera-info-manager \
    ros-humble-compressed-image-transport \
    ros-humble-compressed-depth-image-transport \
    ros-humble-cv-bridge \
    ros-humble-demo-nodes-cpp \
    ros-humble-demo-nodes-py \
    ros-humble-diagnostic-updater \
    ros-humble-example-interfaces \
    ros-humble-foxglove-bridge \
    ros-humble-image-geometry \
    ros-humble-image-pipeline \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-launch-xml \
    ros-humble-launch-yaml \
    ros-humble-launch-testing \
    ros-humble-launch-testing-ament-cmake \
    ros-humble-nav2-bringup \
    ros-humble-nav2-msgs \
    ros-humble-nav2-mppi-controller \
    ros-humble-navigation2 \
    ros-humble-ompl \
    ros-humble-resource-retriever \
    ros-humble-rqt-graph \
    ros-humble-rqt-reconfigure \
    ros-humble-rqt-image-view \
    ros-humble-rviz2 \
    ros-humble-rviz-common \
    ros-humble-rviz-default-plugins \
    ros-humble-sensor-msgs \
    ros-humble-slam-toolbox \
    ros-humble-v4l2-camera \
    ros-humble-vision-opencv \
    ros-humble-vision-msgs \
&& rm -rf /var/lib/apt/lists/* \
&& apt-get clean

# Build Essentials
RUN apt-get update && apt-get install -y \
    build-essential \
    devscripts \
    dh-make \
    quilt \
    fakeroot \
&& rm -rf /var/lib/apt/lists/* \
&& apt-get clean

# Setup rosdep
RUN rosdep init \
    && mkdir -p /etc/ros/rosdep/sources.list.d/ \
    && curl -o /etc/ros/rosdep/sources.list.d/nvidia-isaac.yaml https://isaac.download.nvidia.com/isaac-ros/extra_rosdeps.yaml \
    && echo "yaml file:///etc/ros/rosdep/sources.list.d/nvidia-isaac.yaml" | tee /etc/ros/rosdep/sources.list.d/00-nvidia-isaac.list \
    && rosdep update

# Install additional ROS packages
RUN apt-get update && apt-get install -y \
    ros-humble-camera-info-manager \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-compression-zstd \
    ros-humble-rosbag2-cpp \
    ros-humble-rosbag2-py \
    ros-humble-rosbridge-suite \
    ros-humble-rqt-graph \
    ros-humble-rqt-reconfigure \
    ros-humble-rviz-common \
    ros-humble-rviz-default-plugins \
&& rm -rf /var/lib/apt/lists/* \
&& apt-get clean

# In the future, it would be great to include steps here to download the model
# files for the BEVFusion package AND build the TensorRT engines automatically.
# For now, we will just leave those as manual steps.

# Set the entrypoint to /bin/bash
ENTRYPOINT ["/bin/bash"]

# NOTE: If for some reason you can't get this Dockerfile to build--just use the
# base image above and install ros2 galactic once you run a container from that.