project(ROBOT_BICYCLE CXX C)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${ROBOT_BICYCLE_MODULE_PATH}
    ${ROBOT_BICYCLE_SOURCE_DIR}/cmake/Modules/)

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
    set(CMAKE_CXX_FLAGS "-Wall -Wno-unused-local-typedefs ${ENABLE_CXX11}")
    set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb3")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
endif()

include_directories(${ROBOT_BICYCLE_SOURCE_DIR}/inc
                    ${ROBOT_BICYCLE_SOURCE_DIR}/firmware)

add_subdirectory(${ROBOT_BICYCLE_SOURCE_DIR}/src)

