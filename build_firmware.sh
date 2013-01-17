#!/bin/bash
# To build the firmware, adjust TRGT to point to your toolchain, and set
# USE_FPU in accordence with whether your target MCU has a FPU or not.
make USE_FPU=no VERBOSE_COMPILE=yes TRGT=~/toolchain/bin/arm-none-eabi-
