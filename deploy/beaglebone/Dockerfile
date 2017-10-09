FROM syrianspock:beaglebone-ros
## ^^ first download and build this locally from https://github.com/SyrianSpock/beaglebone-docker-images

USER root

RUN apt-get update && apt-get install -y \
  python-pip \
  python-smbus
RUN pip install pyserial Adafruit_BBIO
RUN git clone https://github.com/adafruit/Adafruit_Python_BNO055.git ~/Adafruit_Python_BNO055 && \
  cd ~/Adafruit_Python_BNO055 && \
  python setup.py install

COPY . /catkin_ws
WORKDIR /catkin_ws

RUN . /opt/ros/indigo/setup.sh && \
    . ./devel/setup.sh && \
    catkin_make

# Overwrite entrypoint w/ our regular one (which does the same thing but without overwriting environment vars)
ENTRYPOINT ["./rep.sh"]
