
include(${CHIBIOS_ROOT}/tools/cmake/chibios.cmake)
include(${CHIBIOS_ROOT}/tools/cmake/ext/chprintf.cmake)
include(${CHIBIOS_ROOT}/tools/cmake/ext/fatfs.cmake)


set(USE_OPT "-O2 -ggdb3 -fomit-frame-pointer -falign-functions=16 -ffunction-sections -fdata-sections -fno-common")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${USE_OPT}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${USE_OPT} -fno-rtti -fno-exceptions -fno-stack-protector -std=c++11")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--gc-sections --static -lc -lnosys")

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(ROBOT_BICYCLE_SRCS
    bitband.h
    chconf.h
    Constants.h
    ffconf.h
    halconf.h
    imu_calibration.cpp
    imu_calibration.h
    Interrupts.cpp
    main.cpp
    mcuconf.h
    MPU6050.cpp
    MPU6050.h
    PeripheralInit.cpp
    PeripheralInit.h
    RearWheel.cpp
    RearWheel.h
    RearWheel_priv.h
    SampleAndControl.cpp
    SampleAndControl.h
    SampleAndControl_priv.h
    Sample.h
    stubs.cpp
    stubs.h
    SystemCommands.cpp
    SystemCommands.h
    textutilities.cpp
    textutilities.h
    VectorTable.cpp
    VectorTable.h
    YawRateController.cpp
    YawRateController.h
    YawRateController_priv.h
)

# Source file properties (e.g. generated) are only visible to
# targets added in the same directory.
nanopb_generate_cpp(PROTO_SRCS PROTO_HDRS "../Test.proto")

set(ROBOT_BICYCLE_DESIGN_DIR "${ROBOT_BICYCLE_SOURCE_DIR}/../design")
set(YRC_GAINS_CPP "${CMAKE_CURRENT_BINARY_DIR}/YawRateController_gains.cpp")
set(CONTROLLERGAINS_H "${CMAKE_CURRENT_BINARY_DIR}/ControllerGains.h")

add_custom_command(OUTPUT ${YRC_GAINS_CPP} ${CONTROLLERGAINS_H}
    COMMAND ${PYTHON3_EXECUTABLE} "${ROBOT_BICYCLE_DESIGN_DIR}/generate_cgains.py"
    COMMAND ${CMAKE_COMMAND} -E rename YawRateController_gains.cpp ${YRC_GAINS_CPP}
    COMMAND ${CMAKE_COMMAND} -E rename ControllerGains.h ${CONTROLLERGAINS_H}
    DEPENDS "${ROBOT_BICYCLE_DESIGN_DIR}/pi_gains.mat"
    "${ROBOT_BICYCLE_DESIGN_DIR}/YawRateController_gains.cpp.in"
    "${ROBOT_BICYCLE_DESIGN_DIR}/ControllerGains.h.in"
    "${ROBOT_BICYCLE_DESIGN_DIR}/parameters.py"
    "${ROBOT_BICYCLE_DESIGN_DIR}/control_tools.py"
    "${ROBOT_BICYCLE_DESIGN_DIR}/controller_data.py"
    "${ROBOT_BICYCLE_DESIGN_DIR}/robotic_bicycle_linear_dynamics_vs_logspeed.npz"
    WORKING_DIRECTORY ${ROBOT_BICYCLE_DESIGN_DIR}
    COMMENT "Generating source and header files for controller gains")

set_source_files_properties(${YRC_GAINS_CPP} ${CONTROLLERGAINS_H}
                            PROPERTIES GENERATED TRUE)
set(GENERATED_SRCS ${PROTO_SRCS} ${YRC_GAINS_CPP} ${CONTROLLERGAINS_H})

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_chibios_executable(robot_bicycle ${ROBOT_BICYCLE_SRCS}
                                     ${CHIBIOS_VARIOUS_SRC}
                                     ${GENERATED_SRCS})
add_flash_target(robot_bicycle)
