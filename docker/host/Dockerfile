FROM ros:humble-ros-base-focal

# using bash
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO humble
ENV USER user
ENV DEBIAN_FRONTEND=noninteractive

RUN \
# install Joystick Driver for ROS2 foxy
    apt-get update \
    && apt-get install -y \
    software-properties-common \
    ros-$ROS_DISTRO-rmw-fastrtps-cpp \
    ros-$ROS_DISTRO-fastrtps \
    ros-$ROS_DISTRO-desktop \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-gazebo-ros-pkgs

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# write source commands to .bashrc -> no need to source afterwards
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc \
########################################################
# Add non root user for development
    && useradd --create-home --shell /bin/bash $USER \
    && echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && /sbin/usermod -aG sudo $USER \
    && echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> /home/$USER/.bashrc \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USER/.bashrc \
    && echo "source /home/$USER/eduart_ws/install/setup.bash" >> /home/$USER/.bashrc \
########################################################
# Clean up
    && apt-get clean

COPY ./edu_robot /home/$USER/eduart_ws/src/edu_robot
COPY ./edu_robot_control /home/$USER/eduart_ws/src/edu_robot_control
COPY ./edu_simulation /home/$USER/eduart_ws/src/edu_simulation

WORKDIR /home/$USER/eduart_ws

RUN export RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && colcon build --symlink-install --executor sequential --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

CMD export RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    && source /home/$USER/eduart_ws/install/setup.bash \
    && ros2 launch edu_simulation eduard.launch.py
