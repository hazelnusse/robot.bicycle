project(ROBOT_BICYCLE_HOST CXX C)
cmake_minimum_required(VERSION 2.8.8)
set(CMAKE_MODULE_PATH ${ROBOT_BICYCLE_HOST_SOURCE_DIR}/cmake/Modules)

# Pass in PYTHON_EXECUTABLE, PROTO_FILES, PROTO_OUTPUT_DIR
if(NOT PYTHON_EXECUTABLE OR NOT PROTO_FILES OR NOT PROTO_OUTPUT_DIR)
    message(FATAL_ERROR "Set values for PYTHON_EXECUTABLE, PROTO_FILES, and PROTO_OUTPUT_DIR")
endif()

find_package(Protobuf REQUIRED)
find_package(Eigen3 3.1.2)

set(NANOPB_DIR "${ROBOT_BICYCLE_HOST_SOURCE_DIR}/firmware/nanopb")
set(NANOPB_GENERATOR_EXECUTABLE "${NANOPB_DIR}/generator/nanopb_generator.py")

set(PROTO_SRCS)
set(PROTO_HDRS)
foreach(proto ${PROTO_FILES})
    get_filename_component(ABS_FIL ${proto} ABSOLUTE)
    get_filename_component(FIL_WE ${proto} NAME_WE)
    get_filename_component(FIL_PATH ${proto} PATH)

    set(PROTO_PB "${PROTO_OUTPUT_DIR}/${FIL_WE}.pb")
    set(PROTO_C "${PROTO_OUTPUT_DIR}/${FIL_WE}.pb.c")
    set(PROTO_H "${PROTO_OUTPUT_DIR}/${FIL_WE}.pb.h")
    set(PROTO_PY "${FIL_PATH}/${FILE_WE}_pb2.py")

    add_custom_command(
      OUTPUT ${PROTO_PB} ${PROTO_PY}
      COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
      ARGS -I${PROTOBUF_INCLUDE_DIR} -I${FIL_PATH} -I${NANOPB_DIR}/generator
           -o${PROTO_PB} --python_out=${FIL_PATH} ${ABS_FIL}
      DEPENDS ${ABS_FIL}
      COMMENT "Running C++ protocol buffer compiler on ${proto}")
    message(STATUS "${FIL_PATH}")

    add_custom_command(
      OUTPUT ${PROTO_C} ${PROTO_H}
      COMMAND ${PYTHON_EXECUTABLE}
      ARGS ${NANOPB_GENERATOR_EXECUTABLE} ${PROTO_PB}
      DEPENDS ${PROTO_PB}
      COMMENT "Running nanopb generator on ${PROTO_PB}")

    set(PROTO_SRCS ${PROTO_SRCS} ${PROTO_C})
    set(PROTO_HDRS ${PROTO_HDRS} ${PROTO_H})
endforeach()

add_custom_target(generate_proto_sources ALL
    DEPENDS ${PROTO_SRCS} ${PROTO_HDRS})

# Set g++ compiler flags if GNU C++ compiler is being used
if(CMAKE_COMPILER_IS_GNUCXX)
    execute_process(COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    if(GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7)
        set(ENABLE_CXX11 "-std=c++11")
    elseif(GCC_VERSION VERSION_GREATER 4.3 OR GCC_VERSION VERSION_EQUAL 4.3)
        set(ENABLE_CXX11 "-std=c++0x")
    else()
        message(FATAL_ERROR "C++ compiler does not support C++11")
    endif()
    set(CMAKE_CXX_FLAGS "-Wall ${CMAKE_CXX_FLAGS} ${ENABLE_CXX11}")
endif()

#include_directories(${ROBOT_BICYCLE_SOURCE_DIR}/inc
#                    ${ROBOT_BICYCLE_SOURCE_DIR}/firmware)

#add_subdirectory(${ROBOT_BICYCLE_SOURCE_DIR}/src)
add_subdirectory(design)

