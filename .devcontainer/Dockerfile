FROM humble:dev

RUN apt-key del F42ED6FBAB17C654
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg


# install gstreamer
RUN apt-get update \
&& apt-get -y install --no-install-recommends \
  ros-humble-hardware-interface \
  ros-humble-controller-manager \
  ros-humble-pluginlib \
  ros-humble-ros2-control \
&& apt-get clean -y \
&& rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    gpiod \
    libgpiod-dev \
    python3-libgpiod \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-xacro \
    ros-humble-ros2-control \
  && rm -rf /var/lib/apt/lists/*
