FROM pablogn/rpi-ros-core-indigo
USER root

RUN apt-get update && apt-get install -y \
  build-essential \
  python-pip \
  python-dev \
  python-smbus \
  git
RUN pip install pyserial
RUN git clone https://github.com/adafruit/Adafruit_Python_BNO055.git ~/Adafruit_Python_BNO055 && \
  cd ~/Adafruit_Python_BNO055 && \
  python setup.py install

COPY . /catkin_ws
WORKDIR /catkin_ws

RUN . /opt/ros/indigo/setup.sh && \
    . ./devel/setup.sh && \
    catkin_make
