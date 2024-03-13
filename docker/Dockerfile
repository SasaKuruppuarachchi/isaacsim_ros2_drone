FROM nvcr.io/nvidia/isaac-sim:2023.1.1

ENV DEBIAN_FRONTEND noninteractive

# install dependencies via apt
ENV DEBCONF_NOWARNINGS yes
RUN set -x && \
  apt-get update -y -qq && \
  apt-get upgrade -y -qq --no-install-recommends && \
  apt-get install -y -qq \
    libyaml-cpp-dev curl software-properties-common \
    vim v4l-utils exfat-* \
    openssh-server \
    terminator dbus-x11 \
    python3-pip && \
  : "remove cache" && \
  apt-get autoremove -y -qq && \
  rm -rf /var/lib/apt/lists/*

RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null && \
    apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main" && \
    apt update && \
    apt install -y -qq cmake

# for ros2
RUN ln -sf /usr/share/zoneinfo/Asia/Tokyo /etc/localtime
RUN apt -y -qq update && apt -y -qq install software-properties-common mesa-utils libglu1-mesa-dev\
  freeglut3-dev mesa-common-dev libglew-dev libglfw3-dev libglm-dev \
  libao-dev libmpg123-dev libasio-dev && \
  add-apt-repository universe && \
  apt -y -qq update && apt install curl -y && \
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
  apt -y -qq update && \
  apt -y -qq upgrade && \
  apt -y -qq install ros-humble-desktop \
    ros-humble-ament-cmake ros-humble-angles ros-humble-controller-manager \
    ros-humble-pluginlib ros-humble-urdf ros-humble-yaml-cpp-vendor ros-humble-joint-state-pub* \
    ros-humble-cv-bridge ros-humble-diagnostic-updater \
    ros-humble-moveit && \
  apt-get install -y -qq ros-humble-xacro ros-humble-realtime-tools ros-humble-control-toolbox ros-humble-ros2-control* \
    ros-humble-teleop-twist-keyboard ros-humble-vision-msgs && \
  : "remove cache" && \
  apt-get autoremove -y -qq && \
  rm -rf /var/lib/apt/lists/*
  
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    lsb-release \
    python3-colcon-ros \
    telnet \
    && apt-get clean

RUN apt install git make cmake python3-pip
RUN pip install kconfiglib jinja2 empy jsonschema pyros-genmsg packaging toml numpy future

# Drone dev
WORKDIR cd /home/ubuntu
RUN git clone https://github.com/PegasusSimulator/PegasusSimulator.git
RUN git clone https://github.com/PX4/PX4-Autopilot.git
WORKDIR cd /home/ubuntu/PX4-Autopilot
RUN git checkout v1.14.1
RUN git submodule update --init --recursive
RUN make px4_sitl_default none


RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
RUN echo 'source /root/colcon_ws/install/setup.sh' >> ~/.bashrc

RUN echo 'export ISAACSIM_PATH="/isaac-sim"' >> ~/.bashrc
RUN echo 'alias ISAACSIM_PYTHON="${ISAACSIM_PATH}/python.sh"' >> ~/.bashrc
RUN echo 'alias ISAACSIM="${ISAACSIM_PATH}/runapp.sh"' >> ~/.bashrc    



ENTRYPOINT ["/bin/bash"]
