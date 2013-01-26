# robot.bicycle

## Hardware
The following is a list of the electrical hardware used on the bicycle:

- Steer Motor Controller: [Accelnet Panel ADP-090-36](http://www.copleycontrols.com/motion/pdf/Accelnet_Panel_ADP.pdf)
- Rear hub motor Controller: [Accelnet Micro Panel ACJ-055-18](http://www.copleycontrols.com/motion/pdf/Accelnet_Micro_Panel.pdf)
- Steer motor: [Teknic M-3441](http://www.teknic.com/files/product_info/N34_Industrial_Grade_Motors_v3.2.pdf)
- Rear hub motor: [Amped Bikes Rear Direct Drive](http://ampedbikes.com/kits.html)
- Wheel Encoders: [US Digital H5-50-N-D](http://usdigital.com/assets/general/119_h5_datasheet_1.pdf)
- Inertial Measurement Unit:
    - [Invensense MPU-6050](http://www.invensense.com/mems/gyro/mpu6050.html)
    - [Sparkfun Electronics Triple Axis Accelerometer & Gyro Breakout - MPU-6050](https://www.sparkfun.com/products/11028)
- Microcontroller:
    - [Olimex STM32 H-407](http://www.olimex.com/dev/pdf/ARM/ST/STM32-H107.pdf)
    - [STM32F407ZG Product page](http://www.st.com/internet/mcu/product/252136.jsp)
    - [STM32F407ZG Datasheet](http://www.st.com/internet/com/TECHNICAL_RESOURCES/TECHNICAL_LITERATURE/DATASHEET/DM00037051.pdf)
    - [STM32F407ZG Reference Manual](http://www.st.com/internet/com/TECHNICAL_RESOURCES/TECHNICAL_LITERATURE/REFERENCE_MANUAL/DM00031020.pdf)
- Quadruple Differential Line Receivers: [TI AM26C32IN](http://www.ti.com/litv/pdf/slls104i)
- JTAG Cable: [Olimex ARM-USB-TINY-H](https://www.olimex.com/Products/ARM/JTAG/ARM-USB-TINY-H/)
- Wireless Radio: [XBee-PRO 802.15.4 extended-range module w/ RPSMA connector](http://www.digi.com/products/model?mid=3270)
- USB to RS232 Cable: [FTDI US232R-10](http://www.ftdichip.com/Support/Documents/DataSheets/Cables/DS_US232R-10_R-100-500.pdf)
- USB to Xbee board [XBee Explorer Dongle](https://www.sparkfun.com/products/9819)

## Wiring Notes
All connections between various components of the robot bicycle are documented
in a [Google Drive spreadsheet](https://docs.google.com/spreadsheet/ccc?key=0Asn6BMg-bB_EdHdMVVBqRTA4Q3IteWdEN1VJOXBDZHc).

## Resources
- Debugging and flashing software [OpenOCD](http://openocd.berlios.de/web/)
- Real time operating system [ChibiOS/RT](http://www.chibios.org/)

## GNU Toolchain
To compile the code that runs on the microcontroller, I've been using two GCC
Arm toolchains to build the firmware.  One is the [GNU Tools for ARM Embedded
Processors](https://launchpad.net/gcc-arm-embedded), which is maintained by
ARM.  The other is the Linaro toolchain, which is updated more frequently but
not necessarily customized for embedded chips.  I maintain a [simple
script](https://github.com/hazelnusse/arm-toolchain) to download and build the
tools in the Linaro toolchain and it seems to work well.

## Build System
To compile the firmware which runs on the bicycle, type:

    $ ./build_firmware.sh

This shell script simply calls the ChibiOS provided Makefile with some
options that affect the code compilation.  You will need to modify this to
point to your toolchain path.

To build the dataprocessing code, ensure you have CMake installed, then type:

    $ mkdir build && cd build
    $ cmake ..
    $ make

## Acknowledgements
This project has been supported in part by NSF Award #0928339.  I am grateful
for the help of Derek Pell, Kenny Koller, Oliver Lee, Kenny Lyons, and the rest
of my lab mates: Bo Fu, Colin Smith, Andrew Kickertz, Jason Moore, Ziqi Yin,
and Gilbert Gede.
