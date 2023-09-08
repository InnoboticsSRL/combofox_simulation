FROM osrf/ros:humble-desktop

# Make sure everything is up to date before building from source
RUN apt-get update \
  && apt-get upgrade -y \
  && apt-get clean

RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    lsb-release \
    python3-colcon-ros \
    python3-colcon-common-extensions \
    ros-$ROS_DISTRO-gazebo-ros ros-$ROS_DISTRO-gazebo-plugins ros-$ROS_DISTRO-gazebo-ros-pkgs \
    xterm \
    ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-* \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-teleop-twist-joy \
    ros-$ROS_DISTRO-teleop-twist-keyboard\
    && apt-get clean

RUN apt-get update && apt-get install -y ros-humble-xacro \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-camera-ros \
    ros-humble-gazebo-plugins \
    ros-humble-moveit-ros-move-group \
    ros-humble-pilz-industrial-motion-planner \
    ros-humble-moveit


RUN mkdir -p /home/ros2_ws/src \
    && cd /home/ros2_ws/src/ 

WORKDIR /home/ros2_ws
ADD src/ src/. 

RUN rosdep fix-permissions && rosdep update \
    && rosdep install --from-paths ./ -i -y --rosdistro humble \
      --ignore-src

RUN  . /opt/ros/humble/setup.sh \
  && colcon build --merge-install

COPY entrypoint.sh /entrypoint.sh
RUN ["chmod", "+x", "/entrypoint.sh"]
ENTRYPOINT ["/entrypoint.sh"]