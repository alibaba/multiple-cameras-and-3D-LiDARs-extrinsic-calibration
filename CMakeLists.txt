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


include_directories(
    src 
    "${CMAKE_CURRENT_SOURCE_DIR}/")


# executable
# if(WITH_VISUALIZATION)
#     message(STATUS "Compilation include viewer.cpp")
#     add_executable(test_stereocalibrator ./demo/test_stereocalibrator.cpp viewer.cpp)
#     target_link_libraries(test_stereocalibrator ${OpenCV_LIBS}  ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES}  ${CERES_LIBRARIES} ${Pangolin_LIBRARIES} Relocalizer calibration)

#     message("--Compilation of test_monocalibrator")
#     add_executable(test_monocalibrator ./demo/test_monocalibrator.cpp viewer.cpp)
#     target_link_libraries(test_monocalibrator ${OpenCV_LIBS}  ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES}  ${CERES_LIBRARIES} ${Pangolin_LIBRARIES} Relocalizer calibration)

#     message("--Compilation of test_techecalibrator")
#     add_executable(test_techecalibrator ./demo/test_techecalibrator.cpp viewer.cpp)
#     target_link_libraries(test_techecalibrator ${OpenCV_LIBS}  ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES}  ${CERES_LIBRARIES} ${Pangolin_LIBRARIES} ${Open3D_LIBRARIES} Relocalizer calibration)
# else(WITH_VISUALIZATION)
#     message(STATUS "Compilation exclude viewer.cpp")
#     add_executable(test_stereocalibrator ./demo/test_stereocalibrator.cpp )
#     target_link_libraries(test_stereocalibrator ${OpenCV_LIBS}  ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES}  ${CERES_LIBRARIES} Relocalizer calibration)

#     message("--Compilation of test_monocalibrator")
#     add_executable(test_monocalibrator ./demo/test_monocalibrator.cpp )
#     target_link_libraries(test_monocalibrator ${OpenCV_LIBS}  ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES}  ${CERES_LIBRARIES} Relocalizer calibration)

#     message("--Compilation of test_techecalibrator")
#     add_executable(test_techecalibrator ./demo/test_techecalibrator.cpp)
#     target_link_libraries(test_techecalibrator ${OpenCV_LIBS}  ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES}  ${CERES_LIBRARIES} ${Open3D_LIBRARIES} Relocalizer calibration)
# endif(WITH_VISUALIZATION)



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
add_executable(test_eval_kalibr ./src/eval_kalibr_result.cpp)
target_link_libraries(test_eval_kalibr ${OpenCV_LIBS}  ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES}  ) 