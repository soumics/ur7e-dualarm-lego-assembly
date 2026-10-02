# Dual-arm LEGO assembly workspace image.
# Base: stock ROS 2 Jazzy desktop-full (Ubuntu 24.04, ships Gazebo Harmonic gz-sim 8).
# Adds: ros2_control + gz_ros2_control, MoveIt 2, Mesa utils, and the rosdep
# dependencies of everything in src/ (UR forks are built from source, not apt).
FROM osrf/ros:jazzy-desktop-full

ARG USERNAME=ros
ARG USER_UID=1001
ARG USER_GID=1001
ENV DEBIAN_FRONTEND=noninteractive

# dist-upgrade first: the base image is an older ROS sync snapshot, and mixing it
# with newly installed packages causes ABI errors (e.g. controller_manager vs
# diagnostic_updater symbol lookup failures inside gz_ros2_control).
RUN apt-get update && apt-get dist-upgrade -y && apt-get install -y --no-install-recommends \
        mesa-utils \
        libgl1-mesa-dri \
        libegl1 \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-gz-ros2-control \
        ros-jazzy-moveit \
        ros-jazzy-moveit-servo \
        ros-jazzy-moveit-configs-utils \
        ros-jazzy-moveit-visual-tools \
        ros-jazzy-moveit-task-constructor-core \
        ros-jazzy-realsense2-description \
        python3-pip \
        python3-venv \
        ffmpeg xdotool wmctrl x11-utils \
        sudo git nano less htop iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# Install dependencies of the source packages. Only package.xml content matters
# here; the sources themselves are bind-mounted at runtime. Re-build the image
# (or run rosdep inside the container) when new dependencies are added.
COPY src /tmp/ws_src
RUN apt-get update \
    && rosdep update --rosdistro jazzy \
    && rosdep install --from-paths /tmp/ws_src --ignore-src --rosdistro jazzy -r -y \
    && rm -rf /tmp/ws_src /var/lib/apt/lists/*

# Assembly planner solver environment, isolated from the system Python: OR-Tools needs
# protobuf >= 6.33 and numpy >= 2, which would break the ROS stack (protobuf 4.21, numpy 1.x
# used by cv_bridge and the Gazebo Python bindings).
RUN python3 -m venv /opt/lego_planner_venv \
    && /opt/lego_planner_venv/bin/pip install --no-cache-dir ortools==9.15.6755

# Non-root user matching the host UID/GID so colcon artifacts on the bind mount
# stay owned by the host user.
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && usermod -aG video,sudo ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER ${USERNAME}
RUN rosdep update --rosdistro jazzy \
    && echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc \
    && echo '[ -f ~/ws/install/setup.bash ] && source ~/ws/install/setup.bash' >> ~/.bashrc \
    && echo 'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash' >> ~/.bashrc

# Workspace defaults. ROS_DOMAIN_ID isolates this project from other ROS 2
# containers on the host network (e.g. drcsim_jazzy). UR_TYPE is the default
# arm model; any ur_description model can be passed explicitly instead.
ENV ROS_DOMAIN_ID=42 \
    UR_TYPE=ur7e \
    RENDER_MODE=gpu

WORKDIR /home/${USERNAME}/ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
