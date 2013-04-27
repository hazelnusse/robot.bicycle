cmake_minimum_required(VERSION 2.8.8)
project(ROBOT_BICYCLE)

set(CMAKE_MODULE_PATH "${ROBOT_BICYCLE_SOURCE_DIR}/../cmake/Modules")
set(CHIBIOS_ROOT "${ROBOT_BICYCLE_SOURCE_DIR}/ChibiOS-RT")
get_filename_component(CHIBIOS_ROOT "${CHIBIOS_ROOT}" ABSOLUTE)

set(CHIBIOS_BOARD_CMAKE_FILE
    "${ROBOT_BICYCLE_SOURCE_DIR}/board/olimex-stm32-h407.cmake")

include(${CHIBIOS_ROOT}/tools/cmake/openocd.cmake)

find_package(PythonInterp 2.7 REQUIRED)
find_package(PythonModule)
find_python_module(jinja2 REQUIRED)
find_python_module(numpy REQUIRED)
find_python_module(scipy REQUIRED)
find_python_module(control REQUIRED)

set(CHIBIOS_VARIOUS_SRC
    ${CHIBIOS_ROOT}/os/various/chprintf.c
    ${CHIBIOS_ROOT}/os/various/chprintf.h
    ${CHIBIOS_ROOT}/os/various/evtimer.c
    ${CHIBIOS_ROOT}/os/various/evtimer.h
    ${CHIBIOS_ROOT}/os/various/shell.c
    ${CHIBIOS_ROOT}/os/various/shell.h
    ${CHIBIOS_ROOT}/os/various/syscalls.c)

add_definitions(-DPB_NO_ERRMSG=1)
#set(PROTO_FILES "${ROBOT_BICYCLE_SOURCE_DIR}/../proto/sample.proto")
#set(PROTO_OUTPUT_DIR ${ROBOT_BICYCLE_BINARY_DIR}/src)
get_filename_component(ROBOT_BICYCLE_HOST_DIR ${ROBOT_BICYCLE_SOURCE_DIR} PATH)
set(PROTO_FILES "${ROBOT_BICYCLE_HOST_DIR}/proto/sample.proto")
set(PROTO_OUTPUT_DIR ${ROBOT_BICYCLE_BINARY_DIR}/src)
set(PROTO_SRCS "${PROTO_OUTPUT_DIR}/sample.pb.c"
    "${ROBOT_BICYCLE_SOURCE_DIR}/nanopb/pb_encode.c")
include_directories(${PROTO_OUTPUT_DIR})

include(ExternalProject)
ExternalProject_Add(ROBOT_BICYCLE_HOST
    PREFIX "${ROBOT_BICYCLE_BINARY_DIR}/host"
    SOURCE_DIR "${ROBOT_BICYCLE_SOURCE_DIR}/.."
    CMAKE_ARGS "-DPYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}"
    "-DPROTO_FILES=${PROTO_FILES}" "-DPROTO_OUTPUT_DIR=${PROTO_OUTPUT_DIR}"
    INSTALL_COMMAND "")

include_directories(${ROBOT_BICYCLE_SOURCE_DIR}/nanopb
                    ${ROBOT_BICYCLE_SOURCE_DIR}/Singleton)
add_subdirectory(${ROBOT_BICYCLE_SOURCE_DIR}/Singleton)
add_subdirectory(${ROBOT_BICYCLE_SOURCE_DIR}/src)

