FROM ros:indigo-ros-base
USER root

RUN apt-get update && apt-get install -y python-pip
RUN git clone https://github.com/openai/gym && cd gym && pip install gym -e .
RUN apt-get install -y python-opengl

COPY . /catkin_ws
WORKDIR /catkin_ws

COPY rep.sh /ros_entrypoint.sh
