FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=humble
ARG USERNAME=loop_dev
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
ARG MACHINE_TYPE=developer

ENV ROS_DISTRO=${ROS_DISTRO}
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV SHELL=/bin/bash

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg \
    lsb-release \
    locales \
    software-properties-common \
    sudo \
  && locale-gen en_US en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && add-apt-repository universe \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo ${UBUNTU_CODENAME}) main" \
    > /etc/apt/sources.list.d/ros2.list \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    can-utils \
    clang-format \
    cmake \
    evtest \
    geographiclib-tools \
    git \
    git-lfs \
    gdb \
    htop \
    iproute2 \
    iputils-ping \
    libboost-chrono-dev \
    libboost-date-time-dev \
    libboost-filesystem-dev \
    libboost-program-options-dev \
    libboost-regex-dev \
    libboost-serialization-dev \
    libboost-system-dev \
    libboost-thread-dev \
    libboost-timer-dev \
    libgeographic-dev \
    libopencv-dev \
    libpcl-dev \
    libserial-dev \
    nano \
    net-tools \
    openssh-client \
    python3-dev \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-serial \
    python3-vcstool \
    python3-venv \
    ruby \
    tmux \
    tmuxinator \
    udev \
    unzip \
    usbutils \
    vim \
    wget \
  && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-ros2-control-test-assets \
    ros-${ROS_DISTRO}-ros2controlcli \
    ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-eigen3-cmake-module \
    ros-${ROS_DISTRO}-generate-parameter-library \
    ros-${ROS_DISTRO}-geographic-msgs \
    ros-${ROS_DISTRO}-gps-msgs \
    ros-${ROS_DISTRO}-gtsam \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-joint-trajectory-controller \
    ros-${ROS_DISTRO}-kdl-parser \
    ros-${ROS_DISTRO}-moveit-configs-utils \
    ros-${ROS_DISTRO}-moveit-kinematics \
    ros-${ROS_DISTRO}-moveit-planners \
    ros-${ROS_DISTRO}-moveit-ros-planning-interface \
    ros-${ROS_DISTRO}-moveit-simple-controller-manager \
    ros-${ROS_DISTRO}-nmea-msgs \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-position-controllers \
    ros-${ROS_DISTRO}-realtime-tools \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-rosidl-default-generators \
    ros-${ROS_DISTRO}-tf2-eigen \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-sensor-msgs \
    ros-${ROS_DISTRO}-topic-tools \
    ros-${ROS_DISTRO}-velocity-controllers \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-xacro \
  && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/requirements.txt
RUN python3 -m pip install --no-cache-dir --upgrade pip \
  && python3 -m pip install --no-cache-dir -r /tmp/requirements.txt

COPY libmavsdk-dev_3.14.0_ubuntu22.04_amd64.deb /tmp/libmavsdk-dev_3.14.0_ubuntu22.04_amd64.deb
RUN dpkg -i /tmp/libmavsdk-dev_3.14.0_ubuntu22.04_amd64.deb \
  || (apt-get update && apt-get install -y --no-install-recommends -f && rm -rf /var/lib/apt/lists/*) \
  && rm -f /tmp/libmavsdk-dev_3.14.0_ubuntu22.04_amd64.deb

RUN if ! getent group ${USER_GID} >/dev/null; then groupadd --gid ${USER_GID} ${USERNAME}; fi \
  && if ! id -u ${USERNAME} >/dev/null 2>&1; then useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} -s /bin/bash; fi \
  && usermod -aG sudo,dialout,video,plugdev ${USERNAME} \
  && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
  && chmod 0440 /etc/sudoers.d/${USERNAME}

RUN rosdep init || true

COPY .devcontainer/loop_ascii.txt /etc/loop_ascii.txt

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc

USER ${USERNAME}
RUN touch /home/${USERNAME}/.hushlogin
RUN printf '\nif [[ $- == *i* ]] && [[ -r /etc/loop_ascii.txt ]]; then\n  printf '"'"'%%b\\n'"'"' "$(cat /etc/loop_ascii.txt)"\nfi\n' \
    >> /home/${USERNAME}/.bashrc

WORKDIR /athena-code
