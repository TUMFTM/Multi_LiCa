FROM ubuntu:22.04

ARG ROS_DISTRO=humble
ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y software-properties-common locales && \
    add-apt-repository universe
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Create a non-root user
ARG USERNAME=local
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  # [Optional] Add sudo support for the non-root user
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  # Cleanup
  && rm -rf /var/lib/apt/lists/* \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc

RUN apt-get update && apt-get install -y -q --no-install-recommends \
    curl \
    nano \
    vim \
    build-essential \
    ca-certificates \
    curl \
    gnupg2 \
    lsb-release \
    python3-pip \
    git \
    cmake \
    make \
    libeigen3-dev \
    apt-utils

# ROS 2
RUN sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && apt-get install -y -q --no-install-recommends \ 
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt-get remove python3-blinker -y

WORKDIR /ros_ws

COPY . /ros_ws/src/multi_lidar_calibration/

ARG TEASERPP_PMC_PIN=a2dfd612a501bca83c47206255dbbff619481f97
ARG TEASERPP_TINYPLY_PIN=c9bb690dfe5e9105961e9e28120c48c9ae084bc6
RUN git clone https://github.com/jingnanshi/pmc.git /tmp/teaserpp-pmc && \
    git -C /tmp/teaserpp-pmc checkout --detach "$TEASERPP_PMC_PIN" && \
    git clone https://github.com/ddiakopoulos/tinyply.git /tmp/teaserpp-tinyply && \
    git -C /tmp/teaserpp-tinyply checkout --detach "$TEASERPP_TINYPLY_PIN" && \
    python3 -m pip install --no-cache-dir --upgrade pip && \
    CMAKE_ARGS="-DFETCHCONTENT_SOURCE_DIR_PMC=/tmp/teaserpp-pmc -DFETCHCONTENT_SOURCE_DIR_TINYPLY=/tmp/teaserpp-tinyply" \
      python3 -m pip install --no-cache-dir \
      ./src/multi_lidar_calibration/TEASER-plusplus && \
    python3 -m pip install --no-cache-dir \
      -r src/multi_lidar_calibration/requirements.txt && \
    rm -rf /tmp/teaserpp-pmc /tmp/teaserpp-tinyply

RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to multi_lidar_calibrator'

ENV QT_DEBUG_PLUGINS=1

CMD [ "bash" ]
