project(ROBOT_BICYCLE_HOST CXX C)
cmake_minimum_required(VERSION 2.8.8)
set(CMAKE_MODULE_PATH ${ROBOT_BICYCLE_HOST_SOURCE_DIR}/cmake/Modules)
set(CMAKE_CXX_FLAGS "-Wall -std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(Protobuf REQUIRED)
find_package(Eigen3 3.1.2)
find_package(Qt4 REQUIRED QtCore QtGui)
find_package(FFTW3 REQUIRED)
include(${QT_USE_FILE})
set(CMAKE_THREAD_PREFER_PTHREAD ON)
find_package(Threads REQUIRED)

if (DEFINED MATLAB_ROOT)
  find_package(Matlab)
else()
  message("MATLAB_ROOT must be defined if you want to build Matlab dependent targets.")
endif()

add_subdirectory(analysis)
add_subdirectory(design)

