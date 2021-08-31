FROM stereolabs/kalibr

# fix ROS GPG Key Expiration Incident
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    curl  \
    wget \
    unzip

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

# google-glog & gflags BLAS & LAPACK & Eigen3 & SuiteSparse and CXSparse (optional)
RUN apt-get install -y libgoogle-glog-dev libgflags-dev \
    libatlas-base-dev libeigen3-dev libsuitesparse-dev 

# eigen3.3.9
WORKDIR /code/eigen3_build
RUN wget -O eigen3.tar.gz https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.gz
RUN tar -xzf eigen3.tar.gz 
RUN cd eigen-3.3.9 && mkdir build && cd build && cmake .. && make install -j$(nproc)

# ceres solver
WORKDIR /code/ceres_build

RUN wget -O ceres.zip https://github.com/ceres-solver/ceres-solver/archive/1.14.0.zip \
    && unzip ceres.zip > /dev/null 2>&1
 
RUN cd ceres-solver-1.14.0 && mkdir build && cd build \
    && cmake .. && make install -j$(nproc)

# yaml-cpp
WORKDIR /code/yaml_build

RUN wget -O yaml-cpp.zip https://github.com/jbeder/yaml-cpp/archive/refs/tags/yaml-cpp-0.6.0.zip \
    && unzip yaml-cpp.zip > /dev/null 2>&1
 
RUN cd yaml-cpp-yaml-cpp-0.6.0/ && mkdir build && cd build \
    && cmake .. -DBUILD_SHARED_LIBS=ON && make install -j$(nproc)

# boost
WORKDIR /code/boost_build

RUN wget -O boost.tar.gz https://boostorg.jfrog.io/artifactory/main/release/1.69.0/source/boost_1_69_0.tar.gz \
    && tar -xvf boost.tar.gz
 
RUN cd boost_1_69_0 && ./bootstrap.sh \
    && ./b2 install

# cctag
WORKDIR /code/cctag_build
RUN apt-get install libpng12-dev libjpeg-dev
RUN git clone https://github.com/fangchuan/CCTag.git && cd CCTag \
    && mkdir build && cd build \
    && cmake .. -DCCTAG_WITH_CUDA:BOOL=OFF -DBUILD_SHARED_LIBS=ON \
    && make install -j$(nproc)


# open3d
# WORKDIR /code/open3d_build
# RUN apt install -y apt-utils
# RUN git clone --recursive https://github.com/isl-org/Open3D.git && \
#     cd Open3D && git checkout v0.10.0 \
#     && git submodule update --init --recursive \
#     && util/scripts/install-deps-ubuntu.sh assume-yes

# RUN cd Open3D && mkdir build && cd build && cmake -DENABLE_GUI=OFF .. \
#     && make install -j$(nproc)


#install scipy
RUN pip install --ignore-installed scipy==1.2.0


RUN /bin/bash -c '. /opt/ros/kinetic/setup.sh'

WORKDIR /code/calibration 
ADD . .

# ln -s /usr/lib/x86_64-linux-gnu/libgtk-x11-2.0.so.0
# ln -s /usr/lib/x86_64-linux-gnu/libfreetype.so.6 /usr/lib/x86_64-linux-gnu/libfreetype.so
# ln -s /usr/lib/x86_64-linux-gnu/libgdk-x11-2.0.so.0
# ln -s /usr/lib/x86_64-linux-gnu/libcairo.so.2
# /usr/lib/x86_64-linux-gnu/libfontconfig.so.1
# /usr/lib/x86_64-linux-gnu/libpangoft2-1.0.so.0
# /usr/lib/x86_64-linux-gnu/libgdk_pixbuf-2.0.so.0
# /usr/lib/x86_64-linux-gnu/libpango-1.0.so.0
# /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL-6.2.so.6.2
# /usr/bin/ld: cannot find -lvtkImagingHybrid
# /usr/bin/ld: cannot find -lvtkIOImage
# /usr/bin/ld: cannot find -lvtkCommonDataModel
# /usr/bin/ld: cannot find -lvtkCommonMath
# /usr/bin/ld: cannot find -lvtkCommonCore
# /usr/bin/ld: cannot find -lvtksys
# /usr/bin/ld: cannot find -lvtkCommonMisc
# /usr/bin/ld: cannot find -lvtkCommonSystem
# /usr/bin/ld: cannot find -lvtkCommonTransforms
# /usr/bin/ld: cannot find -lvtkCommonExecutionModel
# /usr/bin/ld: cannot find -lvtkDICOMParser
# /usr/bin/ld: cannot find -lvtkIOCore
# /usr/bin/ld: cannot find -lvtkmetaio
# /usr/bin/ld: cannot find -lvtkImagingCore
# /usr/bin/ld: cannot find -lvtkRenderingCore
# /usr/bin/ld: cannot find -lvtkCommonColor
# /usr/bin/ld: cannot find -lvtkFiltersExtraction
# /usr/bin/ld: cannot find -lvtkFiltersCore
# /usr/bin/ld: cannot find -lvtkFiltersGeneral
# /usr/bin/ld: cannot find -lvtkCommonComputationalGeometry
# /usr/bin/ld: cannot find -lvtkFiltersStatistics
# /usr/bin/ld: cannot find -lvtkImagingFourier
# /usr/bin/ld: cannot find -lvtkalglib
# /usr/bin/ld: cannot find -lvtkFiltersGeometry
# /usr/bin/ld: cannot find -lvtkFiltersSources
# /usr/bin/ld: cannot find -lvtkInteractionStyle
# /usr/bin/ld: cannot find -lvtkRenderingLOD
# /usr/bin/ld: cannot find -lvtkFiltersModeling
# /usr/bin/ld: cannot find -lvtkIOPLY
# /usr/bin/ld: cannot find -lvtkIOGeometry
# /usr/bin/ld: cannot find -lvtkFiltersTexture
# /usr/bin/ld: cannot find -lvtkRenderingFreeType
# /usr/bin/ld: cannot find -lvtkftgl
# /usr/bin/ld: cannot find -lvtkIOExport
# /usr/bin/ld: cannot find -lvtkRenderingAnnotation
# /usr/bin/ld: cannot find -lvtkImagingColor
# /usr/bin/ld: cannot find -lvtkRenderingContext2D
# /usr/bin/ld: cannot find -lvtkRenderingGL2PS
# /usr/bin/ld: cannot find -lvtkRenderingContextOpenGL
# /usr/bin/ld: cannot find -lvtkRenderingLabel

# ADD docker/calib-entrypoint.sh /calib-entrypoint.sh
# ENTRYPOINT [ "/calib-entrypoint.sh" ]