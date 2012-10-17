project(robot.bike CXX C)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${robot.bike_MODULE_PATH} "${robot.bike_SOURCE_DIR}/cmake/Modules/")

find_package(Eigen3 3.1)

SET(CMAKE_CXX_FLAGS "-std=c++0x -O2")
add_executable(datareader datareader.cc SampleReader.cc Sample.h)

include_directories(${EIGEN3_INCLUDE_DIR})
add_executable(testStateSpace testStateSpace.cc)
