#include "SampleReader.h"
#include "Sample.h"
#include "Constants.h"

#include <limits>

SampleReader::SampleReader(const char * fname)
  : converted_(false)
{
  std::ifstream datafile(fname, std::ios_base::in |
                                std::ios_base::binary);
  if (datafile.is_open()) {
    datafile.seekg(0, std::ios::end);
    std::ifstream::pos_type size = datafile.tellg();
    char * membuff = new char[size];
    datafile.seekg(0, std::ios::beg);
    datafile.read(membuff, size);
    datafile.close();

    if (size % sizeof(Sample)) {
      std::cerr << "Non-integer number of samples in datafile." << std::endl;
    } else {
      unsigned int N = size / sizeof(Sample);
      for (unsigned int i = 0; i < N; ++i) {
        Sample * s = reinterpret_cast<Sample *>(&membuff[i*sizeof(Sample)]);
        samples_.push_back(*s);
      }
      std::cout << size << " bytes read, "
                << samples_.size() << " samples read @ "
                << sizeof(Sample) << " bytes per sample." << std::endl;

    }
    delete [] membuff;
  }
  std::cout << samples_.size() << std::endl;
  samplesConverted_.reserve(samples_.size());
}

std::vector<SampleConverted> SampleReader::Convert()
{
  if (!converted_) {
    uint32_t t0 = samples_[0].SystemTime;
    for (int i = 0; i < samples_.size(); ++i) {
      SampleConverted sc;
      
      // Accelerometer
      sc.Accelerometer[0] = samples_[i].MPU6050[0]
                                              * cd::Accelerometer_sensitivity;
      sc.Accelerometer[1] = samples_[i].MPU6050[1]
                                              * cd::Accelerometer_sensitivity;
      sc.Accelerometer[2] = samples_[i].MPU6050[2]
                                              * cd::Accelerometer_sensitivity;
      // Temperature 
      sc.Temperature = samples_[i].MPU6050[3] * cd::Gyroscope_temp_sensitivity
                       + cd::Gyroscope_temp_offset;
      // Gyroscope
      sc.Gyroscope[0] = samples_[i].MPU6050[4]
                                          * cd::Gyroscope_sensitivity;

      sc.Gyroscope[1] = samples_[i].MPU6050[5]
                                          * cd::Gyroscope_sensitivity;

      sc.Gyroscope[2] = samples_[i].MPU6050[6]
                                          * cd::Gyroscope_sensitivity;

      // Rear wheel angle
      sc.RearWheelAngle = samples_[i].RearWheelAngle * cd::Wheel_rad_per_quad_count;
      // Steer
      sc.SteerAngle = (samples_[i].SteerAngle * cd::Steer_rad_per_quad_count)
                    - ((1 << 15) * cd::Steer_rad_per_quad_count);
      //sc.SteerAngle = (samples_[i].SteerAngle - (1 << 15)) * cd::Steer_rad_per_quad_count;

      // Front wheel angle
      sc.FrontWheelAngle = samples_[i].FrontWheelAngle * cd::Wheel_rad_per_quad_count;

      // Wheel rates and steer rate
      sc.RearWheelRate = encoderRate(samples_[i].SystemState & Sample::RearWheelEncoderB,
                                     samples_[i].RearWheelRate,
                                     cd::Wheel_rad_halfquad_counts_per_sec);
      sc.SteerRate = encoderRate(samples_[i].SystemState & Sample::SteerEncoderB,
                                 samples_[i].SteerRate,
                                 cd::Steer_rad_counts_per_sec);
      sc.FrontWheelRate = encoderRate(samples_[i].SystemState & Sample::FrontWheelEncoderB,
                                      samples_[i].FrontWheelRate,
                                      cd::Wheel_rad_counts_per_sec);

      // Current commands
      sc.I_rw = samples_[i].CCR_rw
                                  * cd::Current_max_rw
                                  / std::pow(2, 16);
      sc.I_steer = samples_[i].CCR_steer
                                     * cd::Current_max_steer
                                     / std::pow(2, 16);

      // Control set points
      sc.RearWheelRate_sp =
                          static_cast<double>(samples_[i].RearWheelRate_sp);
      sc.YawRate_sp =
                          static_cast<double>(samples_[i].YawRate_sp);

      // Convert system time to normal time
      sc.Time = (samples_[i].SystemTime - t0) * cd::timer_dt;
      
      // No conversion needed for Errorcodes
      sc.SystemState = samples_[i].SystemState;

      // Add sc to SampleConverted vector
      samplesConverted_.push_back(sc);
    }
    converted_ = true;
  }
  std::cout << samplesConverted_.size() << std::endl;
  return samplesConverted_;
}


double SampleReader::encoderRate(bool dir, uint32_t counts, double rad_counts_per_sec)
{
  double direction;
  if (dir) {
      direction = -1.0;
  } else {
      direction = 1.0;
  }
  return direction * rad_counts_per_sec / counts;
}
