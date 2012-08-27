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
  samplesConverted_.reserve(samples_.size());
}

std::vector<SampleConverted> SampleReader::Convert()
{
  if (!converted_) {
    uint32_t t0 = samples_[0].SystemTime;
    for (int i = 0; i < samples_.size(); ++i) {
      // Gyroscope
      samplesConverted_[i].Temperature = (samples_[i].Gyroscope[0]
          + cd::Gyroscope_temp_offset) * cd::Gyroscope_temp_sensitivity;

      samplesConverted_[i].Gyroscope[0] = samples_[i].Gyroscope[1]
                                          * cd::Gyroscope_sensitivity;

      samplesConverted_[i].Gyroscope[1] = samples_[i].Gyroscope[2]
                                          * cd::Gyroscope_sensitivity;

      samplesConverted_[i].Gyroscope[2] = samples_[i].Gyroscope[3]
                                          * cd::Gyroscope_sensitivity;

      // Accelerometer
      samplesConverted_[i].Accelerometer[0] = samples_[i].Accelerometer[0]
                                              * cd::Accelerometer_sensitivity;
      samplesConverted_[i].Accelerometer[1] = samples_[i].Accelerometer[1]
                                              * cd::Accelerometer_sensitivity;
      samplesConverted_[i].Accelerometer[2] = samples_[i].Accelerometer[2]
                                              * cd::Accelerometer_sensitivity;

      // Magnetometer
      samplesConverted_[i].Magnetometer[0] = samples_[i].Magnetometer[0]
                                             * cd::Magnetometer_sensitivity;
      samplesConverted_[i].Magnetometer[1] = samples_[i].Magnetometer[1]
                                             * cd::Magnetometer_sensitivity;
      samplesConverted_[i].Magnetometer[2] = samples_[i].Magnetometer[2]
                                             * cd::Magnetometer_sensitivity;

      // Steer
      samplesConverted_[i].SteerAngle = samples_[i].SteerAngle
                                        * cd::Steer_rad_per_quad_count;

      // Wheel rates and steer rate
      samplesConverted_[i].RearWheelRate = cd::Wheel_rad_counts_per_sec
                                           / samples_[i].RearWheelRate;
      samplesConverted_[i].FrontWheelRate = cd::Wheel_rad_counts_per_sec
                                            / samples_[i].FrontWheelRate;
      samplesConverted_[i].SteerRate = cd::Steer_rad_counts_per_sec
                                       / samples_[i].SteerRate;

      // Current commands
      samplesConverted_[i].I_rw = samples_[i].CCR_rw
                                  * cd::Current_max_rw
                                  / std::pow(2, 16);
      samplesConverted_[i].I_steer = samples_[i].CCR_steer
                                     * cd::Current_max_steer
                                     / std::pow(2, 16);

      // Control set points
      samplesConverted_[i].RearWheelRate_sp =
                          static_cast<double>(samples_[i].RearWheelRate_sp);
      samplesConverted_[i].YawRate_sp =
                          static_cast<double>(samples_[i].YawRate_sp);

      // Convert system time to normal time
      samplesConverted_[i].Time = (samples_[i].SystemTime - t0) * 0.005;
      
      // No conversion needed for Errorcodes
      samplesConverted_[i].ErrorCodes = samples_[i].ErrorCodes;
    }
    converted_ = true;
  }
  return samplesConverted_;
}
