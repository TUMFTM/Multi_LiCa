FROM ros:humble

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

RUN apt-get update && apt-get install -y software-properties-common && add-apt-repository universe
RUN apt-get update && apt-get install -y -q --no-install-recommends \
    curl \
    nano \
    vim \
    build-essential \
    ca-certificates \
    curl \
    gnupg2 \
    locales \
    lsb-release \
    python3-pip \
    git \
    cmake \
    make \
    apt-utils

RUN apt-get remove python3-blinker -y

RUN apt-get install ros-$ROS_DISTRO-rmw-cyclonedds-cpp -y
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR /ros_ws

COPY . /ros_ws/src/multi_lidar_calibration

RUN pip install --no-cache-dir --upgrade pip && \
  pip install --no-cache-dir -r src/multi_lidar_calibration/requirements.txt

RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash && \
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to multi_lidar_calibrator'

ENV QT_DEBUG_PLUGINS=1

CMD [ "bash" ]
