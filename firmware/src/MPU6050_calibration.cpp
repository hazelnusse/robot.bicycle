#include "MPU6050.h"

const float MPU6050::gyro_x_bias = -0.12831133410801182f;
const float MPU6050::gyro_y_bias = 0.032857218962598515f;
const float MPU6050::gyro_z_bias = 0.010641128707006363f;
const float MPU6050::dcm[6] = {-0.894519492243436f,
                                    -0.0635181679465503f,
                                    0.0608731305741522f,
                                    0.446568482938378f,
                                    0.997939373875708f,
                                    0.0202846750691697f};
