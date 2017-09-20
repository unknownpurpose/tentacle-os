FROM pablogn:rpi-ros-core-indigo

RUN apt-get update && apt-get install -y \
  build-essential \
  python-pip

RUN pip install pyserial

COPY . /catkin_ws
WORKDIR /catkin_ws

RUN . /opt/ros/indigo/setup.sh && \
    . devel/setup.sh && \
    catkin_make
