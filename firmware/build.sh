#!/bin/bash
# To build the firmware, adjust TRGT to point to your toolchain, and set
# USE_FPU in accordence with whether your target MCU has a FPU or not.
protoc -o Test.pb Test.proto
python2 ./nanopb/generator/nanopb_generator.py Test.pb
make -j8 USE_FPU=yes USE_VERBOSE_COMPILE=yes TRGT=~/toolchain/bin/arm-none-eabi-
