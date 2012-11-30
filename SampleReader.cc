#include <cmath>
#include <limits>
#include "SampleReader.h"
#include "Sample.h"
#include "Constants.h"


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
    uint32_t t_prev = 0;
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
      sc.RearWheelAngle = std::fmod(samples_[i].RearWheelAngle * cd::Wheel_rad_per_quad_count, cd::two_pi);

      // Steer
      sc.SteerAngle = samples_[i].SteerAngle * cd::Steer_rad_per_quad_count;

      // Front wheel angle
      sc.FrontWheelAngle = std::fmod(samples_[i].FrontWheelAngle * cd::Wheel_rad_per_quad_count, cd::two_pi);


      // Rear wheel current command
      double current = samples_[i].CCR_rw * cd::Current_max_rw / std::pow(2, 16);
      if (samples_[i].SystemState & Sample::RearWheelEncoderDir)
        current = -current;
      sc.I_rw = current;
      
      // Steer current command
      current = samples_[i].CCR_steer * cd::Current_max_steer / std::pow(2, 16);
      if (samples_[i].SystemState & Sample::SteerEncoderDir)
        current = -current;
      sc.I_steer = current;

      // Control set points
      sc.RearWheelRate_sp = samples_[i].RearWheelRate_sp;
      sc.YawRate_sp = samples_[i].YawRate_sp;

      // Convert system time to normal time
      uint32_t t = samples_[i].SystemTime;
      if (t < t_prev) {
        std::cout << "Time error." << std::endl;
        std::cout << "t[" << i-1 << "] = " << t_prev << std::endl;
        std::cout << "t[" << i << "] = " << t << std::endl;
      }
      sc.Time = (t - t0) * cd::timer_dt;
      t_prev = t;
      
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
