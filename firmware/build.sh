#!/bin/bash
# To build the firmware, adjust TRGT to point to your toolchain, and set
# USE_FPU in accordence with whether your target MCU has a FPU or not.
protoc -I/usr/include -Inanopb/generator -I. -o Sample.pb Sample.proto 
python2 ./nanopb/generator/nanopb_generator.py Sample.pb
mv Sample.pb.h Sample.pb.c ./src
make -j8 USE_FPU=yes USE_VERBOSE_COMPILE=yes TRGT=~/toolchain/bin/arm-none-eabi-
rm ./src/Sample.pb.c ./src/Sample.pb.h ./Sample.pb

