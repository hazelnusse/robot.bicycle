add_executable(convertsamples convertsamples.cc
                              SampleReader.cc
                              ${ROBOT_BICYCLE_SOURCE_DIR}/firmware/imu_calibration.cpp
                              ${ROBOT_BICYCLE_SOURCE_DIR}/firmware/cgains.cpp)

