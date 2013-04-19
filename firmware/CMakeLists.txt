cmake_minimum_required(VERSION 2.8)
project(ROBOT_BICYCLE)

set(CHIBIOS_ROOT "${ROBOT_BICYCLE_SOURCE_DIR}/ChibiOS-RT")
get_filename_component(CHIBIOS_ROOT "${CHIBIOS_ROOT}" ABSOLUTE)

set(CHIBIOS_BOARD_CMAKE_FILE
    "${ROBOT_BICYCLE_SOURCE_DIR}/board/olimex-stm32-h407.cmake")

include(${CHIBIOS_ROOT}/tools/cmake/openocd.cmake)

set(CHIBIOS_VARIOUS_SRC
    ${CHIBIOS_ROOT}/os/various/chprintf.c
    ${CHIBIOS_ROOT}/os/various/chprintf.h
    ${CHIBIOS_ROOT}/os/various/evtimer.c
    ${CHIBIOS_ROOT}/os/various/evtimer.h
    ${CHIBIOS_ROOT}/os/various/shell.c
    ${CHIBIOS_ROOT}/os/various/shell.h
    ${CHIBIOS_ROOT}/os/various/syscalls.c
)


add_subdirectory("${ROBOT_BICYCLE_SOURCE_DIR}/Singleton")
include_directories(${ROBOT_BICYCLE_SOURCE_DIR}/Singleton
                    ${ROBOT_BICYCLE_SOURCE_DIR}/nanopb)
add_subdirectory("${ROBOT_BICYCLE_SOURCE_DIR}/src")

