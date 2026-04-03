ARG DOCKER_ROS_DISTRO=jazzy
ARG BASE_IMAGE=gazebo-cuda
FROM althack/ros2:${DOCKER_ROS_DISTRO}-dev AS hardware
FROM althack/ros2:${DOCKER_ROS_DISTRO}-gazebo AS gazebo
FROM althack/ros2:${DOCKER_ROS_DISTRO}-cuda-gazebo AS gazebo-cuda

FROM ${BASE_IMAGE:-gazebo-cuda} AS final

# Build arguments
ARG VIRTUALGL_VER="3.1"
ARG TARGETARCH
ARG DOCKER_ROS_DISTRO=jazzy
ARG BASE_IMAGE=gazebo-cuda
ARG USERNAME=ros
ARG WORKSPACE=/home/ros/robot_arm_ws
ARG HOST_UID=1000
ARG HOST_GID=1000
ARG BIN_TO_WRAP="ros2 rviz2 rqt rqt_graph rqt_image_view rqt_plot rqt_console rqt_tf_tree python3"

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    PIP_BREAK_SYSTEM_PACKAGES=1 \
    WORKSPACE=${WORKSPACE}

# Use bash as the default shell to avoid sh/bash double-wrapping quoting issues
SHELL ["/bin/bash", "-c"]

# Install VirtualGL and graphics libraries, and configure GPU-accelerated CLI wrappers (gazebo-cuda only)
RUN /bin/bash -c " \
    if [ '${BASE_IMAGE}' = 'gazebo-cuda' ]; then \
        apt-get update && \
        apt-get install -y --no-install-recommends \
            wget \
            libglvnd0 \
            libgl1 \
            libglx0 \
            libegl1 \
            libxext6 \
            libx11-6 && \
        wget -O /tmp/virtualgl.deb https://github.com/VirtualGL/virtualgl/releases/download/${VIRTUALGL_VER}/virtualgl_${VIRTUALGL_VER}_${TARGETARCH}.deb && \
        apt-get install -y /tmp/virtualgl.deb && \
        rm -f /tmp/virtualgl.deb && \
        rm -rf /var/lib/apt/lists/* ; \
    fi"

# Copy and configure entrypoint before leaving root user
COPY docker/entrypoint.sh /entrypoint.sh

# Remap existing user to match host UID/GID and grant sudo privileges
RUN groupmod --gid ${HOST_GID} ${USERNAME} \
    && usermod --uid ${HOST_UID} --gid ${HOST_GID} ${USERNAME} \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && chown -R ${HOST_UID}:${HOST_GID} /home/${USERNAME}

# Switch to ros user for building and create the workspace
USER ${USERNAME}
RUN mkdir -p ${WORKSPACE}/src
WORKDIR ${WORKSPACE}

# Copy source code and import dependencies
COPY --chown=${USERNAME}:${USERNAME} . ${WORKSPACE}/src/robot_arm

# Configure VGL wrapper functions in user's bashrc (gazebo-cuda only)
RUN if [ "$BASE_IMAGE" = 'gazebo-cuda' ]; then \
        echo "BIN_TO_WRAP=(${BIN_TO_WRAP})" >> ~/.bashrc && \
        echo 'for cmd in "${BIN_TO_WRAP[@]}"; do' >> ~/.bashrc && \
        echo '  eval "function $cmd { vglrun -d /dev/dri/card\${GPU_ID:-0} \$(which $cmd) \"\$@\"; }"' >> ~/.bashrc && \
        echo 'done' >> ~/.bashrc ; \
    fi

# Clone MoveIt2 and related packages into workspace src
RUN git clone --branch jazzy https://github.com/moveit/moveit2 ${WORKSPACE}/src/moveit2 && \
    git clone --branch ros2 https://github.com/moveit/moveit_msgs.git ${WORKSPACE}/src/moveit_msgs && \
    git clone --branch ros2 https://github.com/moveit/moveit_resources.git ${WORKSPACE}/src/moveit_resources

# Update rosdep and install dependencies
RUN sudo apt-get update && \
    rosdep update --rosdistro=${DOCKER_ROS_DISTRO} && \
    rosdep install \
        --rosdistro=${DOCKER_ROS_DISTRO} \
        --from-paths src \
        -iry \
        --os=ubuntu:$(lsb_release --codename | cut -f2) \
        --skip-keys microxrcedds_agent && \
    sudo rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN /bin/bash -c "source /opt/ros/${DOCKER_ROS_DISTRO}/setup.bash && colcon build"

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
