#!/bin/bash
# To build the firmware, adjust TRGT to point to your toolchain, and set
# USE_FPU in accordence with whether your target MCU has a FPU or not.
make -j4 USE_FPU=yes VERBOSE_COMPILE=no TRGT=~/toolchain/bin/arm-none-eabi-
