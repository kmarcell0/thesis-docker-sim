FROM ros:jazzy-ros-base
ENV DEBIAN_FRONTEND=noninteractive

# ROS/Gazebo stack
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-rviz2 ros-jazzy-rqt-image-view \
      ros-jazzy-ament-lint-auto ros-jazzy-ament-lint-common \
      ros-jazzy-ament-cmake-pycodestyle ros-jazzy-ament-cmake-flake8 \
      ros-jazzy-ament-cmake-gtest libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Run in container:
#   rosdep install --from-paths src --ignore-src --simulate
# To install more packages into the container

# Graphics userspace
RUN apt-get update && apt-get install -y --no-install-recommends \
      libgl1-mesa-dri libegl-mesa0 mesa-utils x11-apps sudo bash-completion nano less \
    && rm -rf /var/lib/apt/lists/*

# User setup (with same userid as host, for easy reading changing files)
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
RUN userdel -r ubuntu 2>/dev/null || true \
 && (getent group ${USER_GID} || groupadd --gid ${USER_GID} ${USERNAME}) \
 && useradd --uid ${USER_UID} --gid ${USER_GID} --create-home --shell /bin/bash ${USERNAME} \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME}
USER ${USERNAME}

# Shell setup (bash)
RUN mkdir -p /home/${USERNAME}/.gz \
 && echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc \
 && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> ~/.bashrc
WORKDIR /ws
CMD ["bash"]


