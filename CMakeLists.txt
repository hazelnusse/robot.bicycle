project(dataproccessor CXX)
cmake_minimum_required(VERSION 2.8)
SET(CMAKE_CXX_FLAGS "-std=c++11 -Weffc++")
add_executable(datareader datareader.cc Sample.h)
