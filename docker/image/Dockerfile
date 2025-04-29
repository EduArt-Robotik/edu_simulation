FROM eduartrobotik/eduart-ros-base:humble-1.2.2

# using bash
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO=humble
ENV USER=user
ENV DEBIAN_FRONTEND=noninteractive

USER root
RUN \
# install Joystick Driver for ROS2 foxy
    apt-get update \
    && apt-get install -y \
    software-properties-common \
    ros-$ROS_DISTRO-rmw-fastrtps-cpp \
    ros-$ROS_DISTRO-fastrtps \
    ros-$ROS_DISTRO-desktop \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-hardware-interface \
    ros-$ROS_DISTRO-pluginlib \
    ros-$ROS_DISTRO-tf2-geometry-msgs \
    ros-$ROS_DISTRO-diagnostic-updater \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-laser-geometry

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt-get install -y \
    build-essential \
    ros-dev-tools \
    ros-$ROS_DISTRO-tf2

# write source commands to .bashrc -> no need to source afterwards
RUN apt-get clean 

# set up ros workspace
USER $USER
RUN mkdir -p /home/$USER/ros/src/
COPY ./edu_robot /home/$USER/ros/src/edu_robot
COPY ./edu_robot_control /home/$USER/ros/src/edu_robot_control
COPY ./edu_simulation /home/$USER/ros/src/edu_simulation

# build EduArt packages nodes
WORKDIR /home/$USER/ros
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/$USER/.bashrc \
    && colcon build --symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

# add entry point
COPY --chown=$USER ./entrypoint /home/$USER/
RUN chmod a+x /home/$USER/entrypoint
ENTRYPOINT [ "/home/user/entrypoint" ]
