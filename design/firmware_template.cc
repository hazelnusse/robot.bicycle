#include "firmware_template.h"

namespace firmware_template {

const char * preamble =
R"(#ifndef GAIN_SCHEDULE_H
#define GAIN_SCHEDULE_H
#include <array>
#include <utility>

#include "filter.h"
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
  matrix_t<observer_state_size, observer_state_size> C;
  matrix_t<observer_state_size, observer_input_size> D;
  vector_t<observer_state_size> update(const vector_t<observer_state_size>& x,
                                       const vector_t<observer_input_size>& u) const;
};

struct LQRController {
  matrix_t<plant_model_input_size, plant_model_state_size> C;
  vector_t<plant_model_input_size> update(const vector_t<plant_model_state_size>& x) const;
};

struct PIController {
//  matrix_t<output_size, output_size> Kp;
//  matrix_t<output_size, output_size> Ki;
//  vector_t<output_size> update(const vector_t<output_size>& x,
//                               const vector_t<output_size>& e) const;
  float Kp, Ki;
  float update(float x, float e) const { return x + Ki * constants::loop_period_s * e; }
};

struct controller_t {
  StateEstimator estimator;
  LQRController lqr;
  PIController pi;
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
  void set_state(const vector_t<plant_model_state_size>& state);
  float compute_updated_torque(float torque_prev);
 private:
  bool set_rate(float rate);
  void state_estimate(float torque_prev);
  float lqr_output() const;
  float pi_output() const;

  Sample * s_;
  float rate_;
  float alpha_;
  uint32_t state_estimate_time_;
  vector_t<plant_model_state_size> state_;
  controller_t *ss_lower_, *ss_upper_;
  rt_controller_t r;
  bool pi_control_enabled_;
  first_order_discrete_filter<float> derivative_filter_;
  static const std::array<rt_controller_t, num_gains> schedule_;
};

} // namespace control

#endif // GAIN_SCHEDULE_H
)";

} // namespace firmware_template

