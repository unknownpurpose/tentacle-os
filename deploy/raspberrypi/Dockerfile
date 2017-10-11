FROM pablogn/rpi-ros-core-indigo
USER root

RUN apt-get update && apt-get install -y \
  build-essential \
  python-pip \
  python-dev \
  libopencv-dev \
  python-opencv \
  git
RUN pip install transforms3d numpy

COPY . /catkin_ws
WORKDIR /catkin_ws

RUN . /opt/ros/indigo/setup.sh && \
    . ./devel/setup.sh && \
    catkin_make

# Built for aruco_vision
