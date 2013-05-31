#ifndef GAIN_SCHEDULE_H
#define GAIN_SCHEDULE_H
#include <map>
#include <utility>

#include "matrix.h"
#include "sample.pb.h"

namespace control {
const uint32_t num_gains = {{ NUMGAINS }};
const uint32_t state_size = {{ STATESIZE }};
const uint32_t input_size = {{ INPUTSIZE }};
const uint32_t output_size = {{ OUTPUTSIZE }};

template <int M, int N>
using matrix_t = Matrix<float, int M, int N>;
template <int M>
using vector_t = matrix_t<int M, 1>;

struct StateEstimator {
  matrix_t<state_size, state_size> A;
  matrix_t<state_size, input_size> B;
  vector_t<state_size> update(vector_t<state_size> x,
                              vector_t<input_size> u);
};

struct LQRController {
  matrix_t<output_size, state_size> C;
  vector_t<output_size> update(vector_t<output_size> x);
};

struct PIController {
  matrix_t<output_size, output_size> Ki;
  matrix_t<output_size, output_size> Kp;
  vector_t<output_size> update(vector_t<output_size> x,
                               vector_t<output_size> e);
};

typedef std::tuple<StateEstimator, LQRController, PIController> ss_tuple_t;

class GainSchedule {
 public:
  GainSchedule(Sample& s);
  bool set_rate(float rate);
  const float rate();
  void set_state(const vector_t<state_size>& state);
  float update_estimate_output(float torque_prev);
 private:
  void state_estimate(float torque_prev);
  const float lqr_output();
  const float pi_output();

  Sample s_;
  float rate_;
  float alpha_;
  uint32_t state_estimate_time_;
  vector_t<state_size> state_ {};
  ss_tuple_t ss0_, ss1_;
  const std::map<float, ss_tuple_t> schedule_;
  enum state_space_t {estimator = 0,
                      lqr = 1;
                      pi = 2;};
};

} // namespace control

#endif // GAIN_SCHEDULE_H

