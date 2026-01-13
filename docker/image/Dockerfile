FROM eduartrobotik/eduart-robot-control:0.5.2

# using bash
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO=jazzy
ENV USER=user
ENV DEBIAN_FRONTEND=noninteractive

USER root
RUN \
# install Joystick Driver for ROS2 foxy
    apt-get update \
    && apt-get install -y \
        ros-jazzy-hardware-interface \
        ros-jazzy-laser-geometry \
        ros-jazzy-gz-sim-vendor \
        ros-jazzy-ros-gz-bridge \
        ros-jazzy-ros-gz-sim \
        ros-jazzy-xacro \
        ros-jazzy-rviz2 \
    && apt-get clean

# set up ros workspace
USER $USER
RUN mkdir -p /home/$USER/ros/src/
COPY ./edu_simulation /home/$USER/ros/src/edu_simulation

# build EduArt packages nodes
WORKDIR /home/$USER/ros
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/$USER/ros/install/setup.bash \
    && colcon build --symlink-install --packages-select edu_simulation --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

# add entry point
COPY --chown=$USER ./entrypoint /home/$USER/
RUN chmod a+x /home/$USER/entrypoint
ENTRYPOINT [ "/home/user/entrypoint" ]
