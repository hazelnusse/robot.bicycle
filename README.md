In order to build this, you need GNU Make and an arm-none-eabi toolchain, along
with ChibiOS sources.  The Makefile is hard coded to point to where I keep the
ChibiOS sources, you can override this by:

  $ make CHIBIOS=/path/to/ChibiOS

The build output will be located in the build directory.
