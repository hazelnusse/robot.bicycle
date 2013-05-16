#!/bin/bash
# To build the firmware, adjust TRGT to point to your toolchain, and set
# USE_FPU in accordence with whether your target MCU has a FPU or not.
protoc -I/usr/include -Inanopb/generator -I../proto -o sample.pb --python_out=../proto ../proto/sample.proto
python2 ./nanopb/generator/nanopb_generator.py sample.pb
mv sample.pb.h sample.pb.c ./src
make -j8 USE_FPU=yes USE_VERBOSE_COMPILE=YES BUILD_TEST=NO TRGT=~/toolchain/bin/arm-none-eabi-
rm ./src/sample.pb.c ./src/sample.pb.h ./sample.pb

