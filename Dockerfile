# Use an official ROS image as the base
ARG BASE_IMAGE=osrf/ros
ARG BASE_TAG=humble-desktop
FROM ${BASE_IMAGE}:${BASE_TAG}

ARG ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}

# Install essential dependencies and ROS 2 packages
RUN apt-get update && apt-get install --no-install-recommends -qqy \
    build-essential \
    libatlas-base-dev \
    nano \
    gfortran \
    libblas-dev \
    liblapack-dev \
    git \
    python3-pip \
    python3-dev \
    python3-setuptools \
    python3-wheel \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros2-control \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    ros-${ROS_DISTRO}-qt-gui-cpp \
    ros-${ROS_DISTRO}-rqt-gui-cpp \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-plotjuggler-ros \
    ros-${ROS_DISTRO}-xacro \
    sudo \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Upgrade pip, setuptools, and wheel
RUN python3 -m pip install --no-cache-dir --upgrade pip setuptools wheel

# Install required Python packages
RUN python3 -m pip install --no-cache-dir \
    numpy==1.23.3 \
    matplotlib \
    numpy-quaternion \
    pandas \
    progressbar2 

# Set the workspace
WORKDIR /workspace
COPY . /workspace

# Set bash as the default shell
SHELL ["/bin/bash", "-c"]

RUN pip install --upgrade pip setuptools==59.6.0
# Build the ROS workspace
RUN set -e && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    mkdir -p /workspace && \
    cd /workspace && \
    rm -rf build/ install/ log/ && \
    colcon build --symlink-install || \
    { echo "Build failed. Dumping logs:"; \
      [ -f log/latest_build/build.log ] && cat log/latest_build/build.log || echo "No build log found."; \
      exit 1; }

# Add a non-root user with sudo privileges
ARG MYUID=1000
ARG MYGID=1000
ARG USER=ros

RUN groupadd --gid ${MYGID} ${USER} && \
    useradd --uid ${MYUID} --gid ${MYGID} -m ${USER} && \
    usermod -aG sudo ${USER} && \
    echo "${USER} ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/${USER} && \
    chmod 0440 /etc/sudoers.d/${USER}

# Switch to the non-root user and set the home directory
USER ${USER}
ENV HOME=/home/${USER}

# Ensure ROS and workspace setups are sourced
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ${HOME}/.bashrc && \
    echo "source /workspace/install/setup.bash" >> ${HOME}/.bashrc

# Default command
CMD ["bash"]
