FROM stereolabs/kalibr

# fix ROS GPG Key Expiration Incident
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    curl  \
    wget \
    unzip \
    vim

# update cmake to version 3.14
RUN wget -O cmake-3.14.tar.gz https://cmake.org/files/v3.14/cmake-3.14.0-rc2-Linux-x86_64.tar.gz
RUN tar -xzf cmake-3.14.tar.gz && mv cmake-3.14.0-rc2-Linux-x86_64/ /opt/cmake-3.14
RUN ln -sf /opt/cmake-3.14/bin/* /usr/bin/

# OpenCV
WORKDIR /code/opencv_build
RUN apt-get install -y libavcodec-dev libavformat-dev libswscale-dev libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev \
            libgtk2.0-dev libpng-dev libjpeg-dev libopenexr-dev libtiff-dev  libwebp-dev
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/3.4.1.zip \
    && unzip opencv.zip > /dev/null 2>&1
RUN cd opencv-3.4.1 && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release WITH_QT=ON -DBUILD_opencv_apps=OFF -DBUILD_SHARED_LIBS=OFF -DBUILD_EXAMPLES=OFF  -DBUILD_DOCS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF .. && \
    make install -j$(nproc)
RUN rm -rf /code/opencv_build/

# google-glog & gflags BLAS & LAPACK & Eigen3 & SuiteSparse and CXSparse (optional)
RUN apt-get install -y libgoogle-glog-dev libgflags-dev \
    libatlas-base-dev libeigen3-dev libsuitesparse-dev 

# eigen3.3.9
WORKDIR /code/eigen3_build
RUN wget -O eigen3.tar.gz https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.gz
RUN tar -xzf eigen3.tar.gz 
RUN cd eigen-3.3.9 && mkdir build && cd build && cmake .. && make install -j$(nproc)
RUN rm -rf /code/eigen3_build/

# ceres solver
WORKDIR /code/ceres_build

RUN wget -O ceres.zip https://github.com/ceres-solver/ceres-solver/archive/1.14.0.zip \
    && unzip ceres.zip > /dev/null 2>&1
 
RUN cd ceres-solver-1.14.0 && mkdir build && cd build \
    && cmake .. && make install -j$(nproc)
RUN rm -rf /code/ceres_build/

# yaml-cpp
WORKDIR /code/yaml_build

RUN wget -O yaml-cpp.zip https://github.com/jbeder/yaml-cpp/archive/refs/tags/yaml-cpp-0.6.0.zip \
    && unzip yaml-cpp.zip > /dev/null 2>&1
 
RUN cd yaml-cpp-yaml-cpp-0.6.0/ && mkdir build && cd build \
    && cmake .. -DBUILD_SHARED_LIBS=ON && make install -j$(nproc)

RUN rm -rf /code/yaml_build/

# boost
WORKDIR /code/boost_build

RUN wget -O boost.tar.gz https://boostorg.jfrog.io/artifactory/main/release/1.69.0/source/boost_1_69_0.tar.gz \
    && tar -xvf boost.tar.gz
 
RUN cd boost_1_69_0 && ./bootstrap.sh \
    && ./b2 install
RUN rm -rf /code/boost_build/

# cctag
WORKDIR /code/cctag_build
RUN apt-get install libpng12-dev libjpeg-dev
RUN git clone https://github.com/fangchuan/CCTag.git && cd CCTag \
    && mkdir build && cd build \
    && cmake .. -DCCTAG_WITH_CUDA:BOOL=OFF -DBUILD_SHARED_LIBS=ON \
    && make install -j$(nproc)
RUN rm -rf /code/cctag_build/

# libpng 
WORKDIR /code/libpng_build
RUN wget -O libpng-1.6.37.tar.gz  https://github.com/glennrp/libpng/archive/refs/tags/v1.6.37.tar.gz \
     && tar -xvf libpng-1.6.37.tar.gz
RUN cd libpng-1.6.37 && mkdir build && cd build && cmake .. && make install -j$(nproc)
RUN rm -rf /code/libpng_build/

# open3d
WORKDIR /code/open3d_build
# RUN git clone --recursive https://github.com/isl-org/Open3D.git && \
#     cd Open3D && git checkout v0.10.0 \
#     && git submodule update --init --recursive 
#     # && util/scripts/install-deps-ubuntu.sh assume-yes
RUN git clone --recursive https://github.com/fangchuan/Open3D.git && \
    cd Open3D && git submodule update --init --recursive 

RUN apt-get --yes install python-dev python-tk || true \
            && apt-get --yes install python3-dev python3-tk || true
RUN cd Open3D && mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DENABLE_GUI=OFF .. \
    && make install -j$(nproc)
RUN rm -rf /code/openc3d_build/


#install scipy
RUN pip install --ignore-installed scipy==1.2.0


RUN /bin/bash -c '. /opt/ros/kinetic/setup.sh'

WORKDIR /code/calibration 
ADD . .
RUN mkdir -p build \
    && cd build \
    && rm -rf * \
    && cmake .. \
    && make -j8
# ADD docker/calib-entrypoint.sh /calib-entrypoint.sh
# ENTRYPOINT [ "/calib-entrypoint.sh" ]