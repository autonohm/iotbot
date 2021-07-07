FROM ros:foxy-ros-base-focal

# using bash
SHELL ["/bin/bash", "-c"]

# remove debconf errors (must run before any apt-get calls)
RUN echo "debconf debconf/frontend select Noninteractive" | debconf-set-selections

# install needed linux packages
# apt-get   https://wiki.ubuntuusers.de/apt/apt-get/
# -y        yes to all displayed questions     
RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y apt-utils \
    && apt-get install -y software-properties-common \
    && apt-get install -y git

# install mraa lib for UART communication in shield node
RUN add-apt-repository ppa:mraa/mraa \
    && apt-get update \
    && apt-get install -y libmraa2 libmraa-dev libmraa-java python-mraa python3-mraa mraa-tools

# set up iotbot workspace
RUN mkdir -p /iotbot_ws/src/iotbot

COPY . /iotbot_ws/src/iotbot

WORKDIR /iotbot_ws

# download CRCpp repository for CRC calculations in shield node
RUN cd /iotbot_ws/src/iotbot/iotbot_shield \
    && git clone https://github.com/d-bahr/CRCpp.git

# look for ROS2 package dependencies
# rodsep            https://docs.ros.org/en/independent/api/rosdep/html/commands.html
# -v, --verbose     verbose display
# -y, --default-yes tell the package manager to default to y or fail when installing
# -r                continue installing despite errors
RUN source /opt/ros/foxy/setup.bash \
    && rosdep update \
    && rosdep install -y -r --from-paths src --ignore-src --rosdistro=foxy

# build iotbot_interface first
RUN source /opt/ros/foxy/setup.bash \
    && colcon build --packages-select iotbot_interface
    # && colcon test

# build other nodes after interface to find interface messages in those nodes
RUN source /opt/ros/foxy/setup.bash \
    && source /iotbot_ws/install/setup.bash \
    && colcon build --packages-select iotbot_motion_control \
    && colcon build --packages-select iotbot_shield
    # && colcon test

# write source commands to .bashrc -> no need to source afterwards
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc \
    && echo "source /iotbot_ws/install/setup.bash" >> ~/.bashrc
