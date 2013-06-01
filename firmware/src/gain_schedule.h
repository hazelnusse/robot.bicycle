#ifndef GAIN_SCHEDULE_H
#define GAIN_SCHEDULE_H
#include <map>

#include "matrix.h"
#include "sample.pb.h"

namespace control {
const uint32_t num_gains = 101;
const uint32_t state_size = 4;
const uint32_t input_size = 3;
const uint32_t output_size = 1;

template <int M, int N>
using matrix_t = Matrix<float, M, N>;
template <int M>
using vector_t = matrix_t<M, 1>;

struct StateEstimator {
  matrix_t<state_size, state_size> A;
  matrix_t<state_size, input_size> B;
  vector_t<state_size> update(const vector_t<state_size>& x,
                              const vector_t<input_size>& u) const;
};

struct LQRController {
  matrix_t<output_size, state_size> C;
  vector_t<output_size> update(const vector_t<state_size>& x) const;
};

struct PIController {
  matrix_t<output_size, output_size> Kp;
  matrix_t<output_size, output_size> Ki;
  vector_t<output_size> update(const vector_t<output_size>& x,
                               const vector_t<output_size>& e) const;
};

struct ss_tuple_t {
  StateEstimator estimator;
  LQRController lqr;
  PIController pi;
};

class GainSchedule {
 public:
  GainSchedule();
  float rate() const;
  Sample& sample();
  bool set_sample(Sample& s);
  void set_state(const vector_t<state_size>& state);
  float compute_updated_torque(float torque_prev);
 private:
  bool set_rate(float rate);
  void state_estimate(float torque_prev);
  float lqr_output() const;
  float pi_output() const;

  Sample s_;
  float rate_;
  float alpha_;
  uint32_t state_estimate_time_;
  vector_t<state_size> state_;
  ss_tuple_t *ss_lower_, *ss_upper_;
  static const std::map<float, ss_tuple_t> schedule_;
};

} // namespace control

#endif // GAIN_SCHEDULE_H
