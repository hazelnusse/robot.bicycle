#include <cmath>
#include <limits>
#include "SampleReader.h"
#include "Sample.h"
#include "Constants.h"
#include "imu_calibration.h"
#include "cgains.h"


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
    uint32_t t, t0 = samples_[0].SystemTime;
    std::cout << "N_t0 = " << t0 << std::endl;
    uint32_t t_prev = 0;
    for (unsigned int i = 0; i < samples_.size(); ++i) {
      SampleConverted sc;
      
      // Accelerometer
      sc.Accelerometer[0] = samples_[i].MPU6050[0]
                                              * cd::Accelerometer_sensitivity;
      sc.Accelerometer[1] = samples_[i].MPU6050[1]
                                              * cd::Accelerometer_sensitivity;
      sc.Accelerometer[2] = samples_[i].MPU6050[2]
                                              * cd::Accelerometer_sensitivity;
      // Temperature 
      sc.Temperature = samples_[i].MPU6050[3] * cd::Thermometer_sensitivity
                       + cd::Thermometer_offset;
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
      double current = samples_[i].CCR_rw * cd::Current_max_rw / (reg::PWM_ARR + 1);
      if (samples_[i].SystemState & Sample::RearWheelMotorCurrentDir)
        current = -current;
      sc.I_rw = current;
      
      // Steer current command
      current = samples_[i].CCR_steer * cd::Current_max_steer / (reg::PWM_ARR + 1);
      if (!(samples_[i].SystemState & Sample::SteerMotorCurrentDir))
        current = -current;
      sc.I_steer = current;

      // Control set points
      sc.RearWheelRate_sp = samples_[i].RearWheelRate_sp;
      sc.YawRate_sp = samples_[i].YawRate_sp;

      // Convert system time to normal time
      t = samples_[i].SystemTime;
      if (t < t_prev) {
        std::cout << "Time error." << std::endl;
        std::cout << "t[" << i-1 << "] = " << t_prev << std::endl;
        std::cout << "t[" << i << "] = " << t << std::endl;
      }
      sc.Time = (t - t0) * cd::Rate_Timer_sec_per_count;

      // Convert loop time
      sc.ComputationTime = samples_[i].ComputationTime * cd::Rate_Timer_sec_per_count;
      
      // No conversion needed for Errorcodes
      sc.SystemState = samples_[i].SystemState;

      sc.phi_dot = imu_calibration::phi_dot(samples_[i]);
      if (i == 0) {
        sc.theta_R_dot = 0.0f;
        for (int k = 0; k < 5; ++k)
          sc.x[k] = 0.0f;
        sc.steer_torque = 0.0f;
        sc.steer_current = 0.0f;
      } else { 
        if (i % con::RW_N == 0) {
          const float dtheta = static_cast<int16_t>(samples_[i].RearWheelAngle
                                - samples_[i - con::RW_N].RearWheelAngle)
                       * cf::Wheel_rad_per_quad_count;

          const float dt = sc.Time - (samplesConverted_.end() - con::RW_N)->Time;
          sc.theta_R_dot = dtheta  / dt;
        } else {
          sc.theta_R_dot = samplesConverted_.back().theta_R_dot;
        }

        float input[3] = {samples_[i].YawRate_sp,
                          samples_[i].SteerAngle * cf::Steer_rad_per_quad_count,
                          sc.phi_dot};

        float steer_torque = samplesConverted_.back().steer_torque,
              current;
        // Perform controller state update as long as steer isn't too big
        if ((std::fabs(input[1]) < (45.0f * cf::rad_per_degree)) &&
            cg::state_and_output_update(sc.theta_R_dot, input, sc.x, steer_torque)) {
            current = steer_torque * cf::kT_steer_inv;
        } else {
          current = 0.0f;
          steer_torque = 0.0f;
        }
        sc.steer_torque = steer_torque;
        sc.steer_current = current;
      }

      // Add sc to SampleConverted vector
      samplesConverted_.push_back(sc);
      t_prev = t;
    }
    std::cout << "Total time = " << (t - t0) * cd::Rate_Timer_sec_per_count << std::endl;
    converted_ = true;
  }
  return samplesConverted_;
}

