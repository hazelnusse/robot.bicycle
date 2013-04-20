cmake_minimum_required(VERSION 2.8.8)
project(ROBOT_BICYCLE)

set(CHIBIOS_ROOT "${ROBOT_BICYCLE_SOURCE_DIR}/ChibiOS-RT")
get_filename_component(CHIBIOS_ROOT "${CHIBIOS_ROOT}" ABSOLUTE)

set(CHIBIOS_BOARD_CMAKE_FILE
    "${ROBOT_BICYCLE_SOURCE_DIR}/board/olimex-stm32-h407.cmake")

include(${CHIBIOS_ROOT}/tools/cmake/openocd.cmake)

find_package(PythonInterp 3.3 REQUIRED)
get_filename_component(PYTHON_EXE_DIR ${PYTHON_EXECUTABLE} PATH)
find_program(PYTHON_2TO3_EXECUTABLE 2to3-3.3 PATHS ${PYTHON_EXE_DIR})

# Due to use of toolchain, unset CMAKE_FIND_ROOT_PATH_MODE_INCLUDE so that
# directories on host are searched when find_path() and find_file() are
# called in FindNanopb.cmake.
set(TEMP ${CMAKE_FIND_ROOT_PATH_MODE_INCLUDE})
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE)
set(NANOPB_SRC_ROOT_FOLDER "${ROBOT_BICYCLE_SOURCE_DIR}/nanopb")
include("${NANOPB_SRC_ROOT_FOLDER}/cmake/FindNanopb.cmake")
include_directories(${NANOPB_INCLUDE_DIRS})
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ${TEMP})

#Convert generator script to python 3 so python 2 is not required.
add_custom_target(target_nanpopb_gen_py3 ALL ${PYTHON_2TO3_EXECUTABLE}
    --output-dir=${CMAKE_CURRENT_BINARY_DIR} --nobackups --no-diffs
    --write-unchanged-files --nofix=isinstance --nofix=unicode
    "${NANOPB_SRC_ROOT_FOLDER}/generator/nanopb_generator.py"
    COMMAND ${CMAKE_COMMAND} -E copy
    "${NANOPB_SRC_ROOT_FOLDER}/generator/nanopb_pb2.py"
    "${CMAKE_CURRENT_BINARY_DIR}/nanopb_pb2.py"
    COMMENT "Converting NANOPB_GENERATOR to python 3")
set(NANOPB_GENERATOR_EXECUTABLE
    "${CMAKE_CURRENT_BINARY_DIR}/nanopb_generator.py")


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

