# start docker, enable it, and check if it is running
# systemctl start docker
# systemctl enable docker
# systemctl status docker
FROM ubuntu:18.04

RUN apt-get update
RUN apt-get install -y unzip curl git libeigen3-dev liblapacke-dev libfftw3-dev libqt4-dev libprotobuf-dev protobuf-compiler python-protobuf

RUN git clone https://github.com/thuiskens/robot.bicycle.git
RUN cd robot.bicycle/ && git status
#RUN cd robot.bicycle/ && git submodule set-url Singleton https://github.com/hazelnusse/Singleton.git
RUN cd robot.bicycle/ && sed -i "s/git:/https:/" .gitmodules
RUN cd robot.bicycle/firmware/ && sed -i "s/~//" build.sh
RUN cd robot.bicycle/ && git submodule update --init

ADD "https://developer.arm.com/-/media/Files/downloads/gnu/12.3.Rel1/binrel/arm-gnu-toolchain-12.3.Rel1-x86_64-arm-none-eabi.tar.xz" gcc-arm-none-eabi.tar.xz
# md5sum of the file 00ebb1b70b1f88906c61206457eacb61
RUN md5sum gcc-arm-none-eabi.tar.xz
RUN mkdir toolchain/
RUN cd toolchain/ && pwd
RUN tar -xf gcc-arm-none-eabi.tar.xz --strip-components=1 -C /toolchain
RUN ls toolchain/

RUN cd robot.bicycle/firmware/ && ./build.sh

# Connect the JTAG cable and power the microcontroller

# Connect to the Olimex
#~/robot.bicycle/firmware$ openocd -f openocd.cfg

# Open another shell instance

# Start the debugger
#~/toolchain/bin/arm-none-eabi-gdb -tui build/robot.bike.elf
