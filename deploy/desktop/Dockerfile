FROM ros:indigo-ros-base

RUN apt-get update && apt-get install -y \
  build-essential

COPY . /catkin_ws
WORKDIR /catkin_ws

RUN . /opt/ros/indigo/setup.sh && \
    . devel/setup.sh && \
    catkin_make
