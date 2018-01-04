FROM ioup:opencv
USER root

COPY . /catkin_ws
WORKDIR /catkin_ws

RUN . /opt/ros/indigo/setup.sh && \
    . devel/setup.sh && \
    catkin_make

COPY rep.sh /ros_entrypoint.sh
