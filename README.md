=============
robot.bicycle
=============

Hardware
--------
The following is a list of the electrical hardware used on the bicycle:

- Steer Motor Controller: `Accelnet Panel ADP-090-36 <http://www.copleycontrols.com/motion/pdf/Accelnet_Panel_ADP.pdf>`_
- Rear hub motor Controller: `Accelnet Micro Panel ACJ-055-18 <http://www.copleycontrols.com/motion/pdf/Accelnet_Micro_Panel.pdf>`_
- Steer motor: `Teknic M-3441 <http://www.teknic.com/files/product_info/N34_Industrial_Grade_Motors_v3.2.pdf>`_
- Rear hub motor: `Amped Bikes Rear Direct Drive <http://ampedbikes.com/kits.html>`_
- Wheel Encoders: `US Digital H5-50-N-D <http://usdigital.com/assets/general/119_h5_datasheet_1.pdf>`_
- Inertial Measurement Unit:
    - `Invensense MPU-6050 http://www.invensense.com/mems/gyro/mpu6050.html>`
    - `Sparkfun Electronics Triple Axis Accelerometer & Gyro Breakout - MPU-6050` <https://www.sparkfun.com/products/11028>`
- Microcontroller:
    - `Olimex STM32 H-407 <http://www.olimex.com/dev/pdf/ARM/ST/STM32-H107.pdf>`_
    - `STM32F407ZG Manufacturer page <http://www.st.com/internet/mcu/product/252136.jsp>`
    - `STM32F407ZG Datasheet <http://www.st.com/internet/com/TECHNICAL_RESOURCES/TECHNICAL_LITERATURE/DATASHEET/DM00037051.pdf>`
    - `STM32F407ZG Reference Manual <http://www.st.com/internet/com/TECHNICAL_RESOURCES/TECHNICAL_LITERATURE/REFERENCE_MANUAL/DM00031020.pdf>`
- Quadruple Differential Line Receiver: `TI AM26C32IN <http://www.ti.com/litv/pdf/slls104i>`_
- Octal Buffer/Driver: `TI SN74LVC244A <http://www.ti.com/lit/gpn/sn74lvc244a>`_
- JTAG Cable: `Olimex ARM-USB-TINY-H <http://www.olimex.com/dev/arm-usb-tiny-h.html>`_
- Wireless Radio: `XBee Pro <http://ftp1.digi.com/support/documentation/90000982_B.pdf>`_
- USB to RS232 Cable: `FTDI US232R-10 <http://www.ftdichip.com/Support/Documents/DataSheets/Cables/DS_US232R-10_R-100-500.pdf>`_
- USB to 3.3V TTL Cable: `TTL-232RG-VREG3V3-WE <http://www.ftdichip.com/Support/Documents/DataSheets/Cables/DS_TTL-232RG_CABLES.pdf>`_

Wiring Notes
------------
All connections between various components of the robot bicycle are documented
in a Google Drive spreadsheet:

- `Wiring notes: <https://docs.google.com/spreadsheet/ccc?key=0Asn6BMg-bB_EdHdMVVBqRTA4Q3IteWdEN1VJOXBDZHc>`_


Resources
---------
- `OpenOCD:  <http://openocd.berlios.de/web/>`_
- `ChibiOS/RT: <http://www.chibios.org/>`_

GNU Toolchain
-------------
I've been using two GCC Arm toolchains to build the firmware.  One is the `GNU
Tools for ARM Embedded Processors <https://launchpad.net/gcc-arm-embedded>`,
which is maintained by ARM.  The other is the Linaro toolchain, which is
updated more frequently but not necessarily customized for embedded chips.  I
maintain a simple script to download and build the tools in the Linaro
toolchain and it seems to work well.

- https://github.com/hazelnusse/arm-toolchain

Build System
------------
To compile the firmware which runs on the bicycle, type:

  $ ./build_firmware.sh

To build the dataprocessing code, ensure you have CMake installed, then type:

  $ mkdir build && cd build
  $ cmake ..
  $ make

Acknowledgements
----------------
This project has been supported in part by NSF Award #0928339.  I am grateful
for the help of Derek Pell, Kenny Koller, Oliver Lee, Kenny Lyons, and the rest
of my lab mates: Bo Fu, Colin Smith, Andrew Kickertz, Jason Moore, Ziqi Yin,
and Gilbert Gede.
