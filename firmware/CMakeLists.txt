cmake_minimum_required(VERSION 2.8.8)
project(ROBOT_BICYCLE)

set(CMAKE_MODULE_PATH "${ROBOT_BICYCLE_SOURCE_DIR}/../cmake/Modules")
set(CHIBIOS_ROOT "${ROBOT_BICYCLE_SOURCE_DIR}/ChibiOS-RT")
get_filename_component(CHIBIOS_ROOT "${CHIBIOS_ROOT}" ABSOLUTE)

set(CHIBIOS_BOARD_CMAKE_FILE
    "${ROBOT_BICYCLE_SOURCE_DIR}/board/olimex-stm32-h407.cmake")

include(${CHIBIOS_ROOT}/tools/cmake/openocd.cmake)

find_package(PythonInterp 3.2 REQUIRED)
get_filename_component(PYTHON_EXE_DIR ${PYTHON_EXECUTABLE} PATH)
find_package(PythonModule)
find_python_module(jinja2 REQUIRED)
set(PYTHON3_EXECUTABLE ${PYTHON_EXECUTABLE})

# Python 2 is used for protobuf generation.
set(PYTHON_EXECUTABLE PYTHON_EXECUTABLE-NOTFOUND)
find_package(PythonInterp 2.7 REQUIRED)

# Due to use of toolchain, unset CMAKE_FIND_ROOT_PATH_MODE_INCLUDE so that
# directories on host are searched when find_path() and find_file() are
# called in FindNanopb.cmake.
set(TEMP ${CMAKE_FIND_ROOT_PATH_MODE_INCLUDE})
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE)
set(NANOPB_SRC_ROOT_FOLDER "${ROBOT_BICYCLE_SOURCE_DIR}/nanopb")
find_package(Nanopb)
include_directories(${NANOPB_INCLUDE_DIRS})
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ${TEMP})

set(CHIBIOS_VARIOUS_SRC
    ${CHIBIOS_ROOT}/os/various/chprintf.c
    ${CHIBIOS_ROOT}/os/various/chprintf.h
    ${CHIBIOS_ROOT}/os/various/evtimer.c
    ${CHIBIOS_ROOT}/os/various/evtimer.h
    ${CHIBIOS_ROOT}/os/various/shell.c
    ${CHIBIOS_ROOT}/os/various/shell.h
    ${CHIBIOS_ROOT}/os/various/syscalls.c
)

include_directories(${ROBOT_BICYCLE_SOURCE_DIR}/nanopb
                    ${ROBOT_BICYCLE_SOURCE_DIR}/Singleton)
add_subdirectory(${ROBOT_BICYCLE_SOURCE_DIR}/Singleton)
add_subdirectory(${ROBOT_BICYCLE_SOURCE_DIR}/src)

