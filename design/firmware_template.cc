#include "firmware_template.h"

namespace firmware_template {

const char * preamble =
R"(#ifndef GAIN_SCHEDULE_H
#define GAIN_SCHEDULE_H
#include <array>
#include <utility>

#include "constants.h"
#include "matrix.h"
#include "sample.pb.h"

namespace control {

)";

const char * postamble = 
R"(template <int M, int N>
using matrix_t = Matrix<float, M, N>;
template <int M>
using vector_t = matrix_t<M, 1>;

struct StateEstimator {
  matrix_t<observer_state_size, observer_state_size> A;
  matrix_t<observer_state_size, observer_input_size> B;
  //matrix_t<observer_state_size, observer_state_size> C;
  //matrix_t<observer_state_size, observer_input_size> D;
  vector_t<observer_state_size> update(const vector_t<observer_state_size>& x,
                                       const vector_t<observer_input_size>& u) const;
//  vector_t<observer_output_size> output(const vector_t<observer_state_size>& x,
//                                        const vector_t<observer_input_size>& u) const;
};

struct LQRController {
  matrix_t<plant_model_input_size, plant_model_state_size> C;
  vector_t<plant_model_input_size> update(const vector_t<plant_model_state_size>& x) const;
};

struct PIController {
  float Kp, Ki;
  float update(float e, float x) const { return Kp * e + Ki * x; }
};

struct YawRateMeasurement {
  matrix_t<plant_model_input_size, plant_model_state_size> C;
};

struct controller_t {
  StateEstimator estimator;
  LQRController lqr;
  PIController pi;
  YawRateMeasurement yr;
};

struct rt_controller_t {
  float rate;
  controller_t controller;
  bool operator<(const rt_controller_t& rhs) const;
};


class GainSchedule {
 public:
  GainSchedule();
  float rate() const;
  bool set_sample(Sample& s);
  void set_state(float phi, float delta, float phi_dot, float delta_dot);
  float compute_updated_torque(float torque_prev);
 private:
  bool set_rate(float rate);
  void state_estimate(float torque_prev);
  float lqr_output() const;
  float pi_output();

  Sample * s_;
  float rate_;
  float alpha_;
  float integrator_state_;
  uint32_t state_estimate_time_;
  vector_t<observer_state_size> state_;
  controller_t *ss_lower_, *ss_upper_;
  rt_controller_t r;
  static const std::array<rt_controller_t, num_gains> schedule_;
};

} // namespace control

#endif // GAIN_SCHEDULE_H
)";

} // namespace firmware_template

