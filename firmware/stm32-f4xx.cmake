
# gcc ARMCMx common
include(${CHIBIOS_CMAKE_DIR}/ports/gcc/armcmx.cmake)

# source files
set(CHIBIOS_SRC
    ${CHIBIOS_SRC}
    ${CHIBIOS_ROOT}/os/ports/GCC/ARMCMx/STM32F4xx/cmparams.h
    ${CHIBIOS_ROOT}/os/ports/GCC/ARMCMx/STM32F4xx/vectors.c
)
include_directories(
    ${CHIBIOS_ROOT}/os/ports/GCC/ARMCMx/STM32F4xx
)

# set the type of CPU
set(CPU_FLAGS "-mcpu=cortex-m4 -mthumb -mno-thumb-interwork")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CPU_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CPU_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${CPU_FLAGS} -nostartfiles")

# Definitions introduced by ChibiOS Makefile that we assume we need to copy
add_definitions("-DTHUMB_PRESENT -DTHUMB_NO_INTERWORKING -DTHUMB")
# set the linker script
set(DEFAULT_LINKER_SCRIPT "${ROBOT_BICYCLE_SOURCE_DIR}/STM32F407xG_ram_vectors.ld")

# option to use the hardware FPU
option(CHIBIOS_USE_HARDWARE_FPU "Use hardware FPU" TRUE)
if(CHIBIOS_USE_HARDWARE_FPU)
    add_definitions("-DCORTEX_USE_FPU=TRUE")
    set(FPU_FLAGS "-march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${FPU_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${FPU_FLAGS}")
    # Add path to linker flags so CMake will find the right libraies
    #set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${FPU_FLAGS} -L${TOOLCHAIN_DIR}/${TOOLCHAIN}/lib/armv7e-m/fpu")
else()
    add_definitions("-DCORTEX_USE_FPU=FALSE")
endif()

