FROM ros:indigo-ros-base
USER root

ENV NUM_CORES 2

RUN apt-get update && apt-get install -y \
  build-essential \
  python-pip \
  wget unzip \
  # libjpeg8-dev libtiff5-dev libjasper-dev libpng12-dev \
  libjpeg-dev libtiff-dev libpng-dev libjasper-dev \
  libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
  libxvidcore-dev libx264-dev \
  libgtk-3-dev \
  libatlas-base-dev gfortran
  # python$PYTHON_VERSION-dev wget unzip \
  #                      build-essential cmake git pkg-config libatlas-base-dev gfortran \
  #                      libjasper-dev libgtk2.0-dev libavcodec-dev libavformat-dev \
  #                      libswscale-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libv4l-dev

RUN pip install numpy transforms3d

RUN wget https://github.com/opencv/opencv/archive/3.3.0.zip -O opencv3.zip && \
   unzip -q opencv3.zip && mv /opencv-3.3.0 /opencv
RUN wget https://github.com/opencv/opencv_contrib/archive/3.3.0.zip -O opencv_contrib3.zip && \
   unzip -q opencv_contrib3.zip && mv /opencv_contrib-3.3.0 /opencv_contrib
RUN mkdir /opencv/build
WORKDIR /opencv/build
RUN cmake -D CMAKE_BUILD_TYPE=RELEASE \
  -D BUILD_PYTHON_SUPPORT=ON \
  -D CMAKE_INSTALL_PREFIX=/usr/local \
  -D INSTALL_C_EXAMPLES=ON \
  -D INSTALL_PYTHON_EXAMPLES=ON \
  -D OPENCV_EXTRA_MODULES_PATH=/opencv_contrib/modules \
  -D BUILD_EXAMPLES=ON \
  -D BUILD_NEW_PYTHON_SUPPORT=ON \
  -D WITH_IPP=OFF \
  -D WITH_V4L=ON ..
RUN make -j$NUM_CORES
RUN make install
RUN ldconfig
