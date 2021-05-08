cmake_minimum_required(VERSION 3.0)
project(kalibr_calibration)

add_definitions(-std=c++11)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -Wno-unused-but-set-variable")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -g -Wno-unknown-pragmas -Wall ")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -Wno-unused-but-set-variable")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -Wno-unknown-pragmas -Wall")

option(WITH_VISUALIZATION "Use pangolin to visualize odometry" OFF)
option(BUILD_LIDAR_CALIBRATOR "Build Lidar calibration programs or not" OFF)

# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
MESSAGE(STATUS "OpenCV version is " ${OpenCV_VERSION})

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


# use Pangolin or not
if(WITH_VISUALIZATION)
    message(STATUS "Use Pangolin to visualization camera trajectory")
    add_definitions(-DVISUALIZE_TRAJECTORY)
    # pangolin
    find_package(Pangolin REQUIRED)
    include_directories(${Pangolin_INCLUDE_DIR})
else(WITH_VISUALIZATION)
    message(STATUS "Disable Pangolin to visualization camera trajectory")
endif()



# openmp
find_package( OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# # ceres
# find_package(Ceres REQUIRED)
# include_directories(${CERES_INCLUDE_DIRS})

# BOOST
find_package(Boost REQUIRED date_time chrono thread serialization system filesystem atomic program_options timer)
include_directories(${Boost_INCLUDE_DIRS})

# find yaml-cpp
find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})

#find Open3D
find_package(Open3D REQUIRED)
include_directories(${Open3D_INCLUDE_DIRS})

# find glog
# find_package(glog REQUIRED)
message("glog found : ${GLOG_INCLUDE_DIRS}")
include_directories(${GLOG_INCLUDE_DIRS})

link_libraries(${OpenCV_LIBS}  ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES} glog ${Open3D_LIBRARIES})

# make filesystem and io operation as libraries
add_library(common SHARED
    src/FileSystemTools.cpp
    src/YamlFileIO.cpp
)

# # make Relocalizer as a library
# add_library(Relocalizer SHARED relocalizer.cpp pnp_solver.cpp common.cpp )
# target_link_libraries(Relocalizer ${OpenCV_LIBS}  ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES})

# # make calibration as a library
# add_library(calibration SHARED camera.cpp target_board.cpp mono_calibration.cpp stereo_calibration.cpp teche_calibration.cpp params_config.cpp cost_function.h)
# target_link_libraries(calibration ${OpenCV_LIBS}  ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES} ${CERES_LIBRARIES} Relocalizer)

include_directories(
    inc 
    "${CMAKE_CURRENT_SOURCE_DIR}/")

# executable
# message("--Compilation of cctag detection")
# add_executable(cctag_detector_for_slam ./demo/cctag_detector.cpp)
# target_link_libraries(cctag_detector_for_slam ${OpenCV_LIBS}  ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES} ) 


# message("--Compilation of image undistor")
# add_executable(image_undistor ./demo/image_undistor.cpp)
# target_link_libraries(image_undistor ${OpenCV_LIBS}  ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES} ) 

# message("--Compilation of extrinsic evaluator")
# add_executable(test_eval_extrinsic ./demo/test_eval_extrinsic)
# target_link_libraries(test_eval_extrinsic calibration ) 

message("--Compilation of intrinsic evaluator")
add_executable(test_eval_kalibr ./test/test_eval_kalibr.cpp)
target_link_libraries(test_eval_kalibr ${OpenCV_LIBS}  ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES} glog common ) 

message("--Compilation of lidar2lidar calibration")
add_executable(test_lidar2lidar_calib ./test/test_lidar2lidar_calibration.cpp)
target_link_libraries(test_lidar2lidar_calib ${OpenCV_LIBS}  ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES} glog ${Open3D_LIBRARIES} common)

message("--Compilation of lidar2cam calibration")
add_executable(test_lidar2cam_calibration ./test/test_lidar2cam_calibration.cpp)
target_link_libraries(test_lidar2cam_calibration ${OpenCV_LIBS}  ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES} glog ${Open3D_LIBRARIES} common)
