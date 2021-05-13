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

# ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

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
include_directories(${GLOG_INCLUDE_DIRS})

include_directories(
    inc 
    "${CMAKE_CURRENT_SOURCE_DIR}/")

link_libraries(${OpenCV_LIBS}  ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES} glog ${Open3D_LIBRARIES} ${CERES_LIBRARIES})

# make filesystem and io operation as libraries
add_library(common SHARED
    src/FileSystemTools.cpp
    src/YamlFileIO.cpp
)

# make Relocalizer as a library
add_library(relocalizer SHARED 
            src/relocalizer.cpp 
            src/pnp_solver.cpp 
            src/common.cpp )
target_link_libraries(relocalizer ${OpenCV_LIBS}  ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES})

# make calibration as a library
add_library(calibration SHARED 
            src/camera.cpp 
            src/target_board.cpp 
            src/mono_calibration.cpp 
            src/stereo_calibration.cpp 
            src/teche_calibration.cpp 
            src/params_config.cpp 
            inc/cost_function.h)
target_link_libraries(calibration ${OpenCV_LIBS}  ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES} ${CERES_LIBRARIES} relocalizer)


# executable
message("--Compilation of cctag detection")
add_executable(test_detect_cctag ./test/test_detect_cctag.cpp)
target_link_libraries(test_detect_cctag ${OpenCV_LIBS}  ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES} glog common) 

message("--Compilation of image undistor")
add_executable(test_undist_img ./test/test_undist_image.cpp)
target_link_libraries(test_undist_img ${OpenCV_LIBS}  ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES} glog common) 

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

message("--Compilation of extrinsic evaluation")
add_executable(test_eval_extrinsics ./test/test_eval_extrinsics)
target_link_libraries(test_eval_extrinsics ${OpenCV_LIBS}  ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES} glog  common)

message("--Compilation of converting kalibr result")
add_executable(test_cvt_kalibr_result ./test/test_cvt_kalibr_result.cpp)
target_link_libraries(test_cvt_kalibr_result ${OpenCV_LIBS}  ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES})

message("--Compilation of monocamera calibration")
add_executable(test_mono_calibration ./test/test_monocalibrator.cpp)
target_link_libraries(test_mono_calibration ${OpenCV_LIBS}  ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES} glog ${Open3D_LIBRARIES} common calibration)

message("--Compilation of panoramic camera calibration")
add_executable(test_teche_calibration ./test/test_techecalibrator.cpp)
target_link_libraries(test_teche_calibration ${OpenCV_LIBS}  ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES} glog ${Open3D_LIBRARIES} common calibration)
