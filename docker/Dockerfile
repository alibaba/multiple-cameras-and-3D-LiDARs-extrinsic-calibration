FROM stereolabs/kalibr

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    curl  \
    wget \
    unzip

# update cmake to version 3.14
RUN wget -O cmake-3.14.tar.gz https://cmake.org/files/v3.14/cmake-3.14.0-rc2-Linux-x86_64.tar.gz
RUN tar -xzvf cmake-3.14.tar.gz && mv cmake-3.14 /opt/cmake-3.14
RUN ln -sf /opt/cmake-3.14/bin/* /usr/bin/


# ceres solver
WORKDIR /code/ceres_build

RUN wget -O ceres.zip https://github.com/ceres-solver/ceres-solver/archive/1.14.0.zip \
    && unzip ceres.zip
 
RUN cd ceres-solver-1.14.0 && mkdir build && cd build \
    && cmake .. && make install -j4

# yaml-cpp
WORKDIR /code/yaml_build

RUN wget -O yaml-cpp.zip https://github.com/jbeder/yaml-cpp/archive/refs/tags/yaml-cpp-0.6.0.zip \
    && unzip yaml-cpp.zip
 
RUN cd yaml-cpp && mkdir build && cd build \
    && cmake .. -DYAML_BUILD_SHARED_LIBS=ON && make install -j4

# boost
WORKDIR /code/boost_build

RUN wget -O boost.tar.gz https://boostorg.jfrog.io/artifactory/main/release/1.69.0/source/boost_1_69_0.tar.gz \
    && tar -xvf boost.tar.gz
 
RUN cd boost && ./bootstrap.sh \
    && ./b2 install

# cctag
WORKDIR /code/cctag_build
RUN apt-get install libpng12-dev libjpeg-dev
RUN git clone https://github.com/fangchuan/CCTag.git && cd CCTag \
    && mkdir build && cd build \
    && cmake .. -DCCTAG_WITH_CUDA:BOOL=OFF -DBUILD_SHARED_LIBS=ON \
    && make -j$(nproc)


# open3d
WORKDIR /code/open3d_build

RUN git clone --recursive https://github.com/intel-isl/Open3D && git checkout v0.10.0 \
    && git submodule update --init --recursive

RUN cd Open3D && util/scripts/install_deps_ubuntu.sh

RUN mkdir build && cd build && cmake .. \
    && make install -j$(nproc)


ADD . .

RUN /bin/bash -c '. /opt/ros/kinetic/setup.sh'

ADD docker/calib-entrypoint.sh /calib-entrypoint.sh
ENTRYPOINT [ "/calib-entrypoint.sh" ]