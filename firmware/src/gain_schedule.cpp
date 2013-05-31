#include "gain_schedule.h"

namespace control {
vector_t<state_size> StateEstimator::update(vector_t<state_size> x,
                                            vector_t<input_size> u) {
  return A * x + B * u;
}
vector_t<output_size> LQRController::update(vector_t<output_size> x) {
  return C * x;
}
vector_t<output_size> PIController::update(vector_t<output_size> x,
                                           vector_t<output_size> e) {
  return Kp * x + Ki * e;
}

GainSchedule::GainSchedule(Sample& s) : s_(s) { };

bool GainSchedule::set_rate(float rate)
{
  bool valid = true;
  rate_ = rate;

  auto it = schedule_.lower_bound(rate);
  if (it == schedule_.end()) {
    s_.estimator.theta_R_dot_upper = NAN;
    s_.estimator.theta_R_dot_lower = --it->first;
    valid = false;
  } else {
    s_.estimator.theta_R_dot_upper = it->first;
    ss1_ = it->second;
    if (it->first == rate) {
      s_.estimator.theta_R_dot_lower = it->rate;
      ss0_ = it->second;
    } else if (--it == schedule_.begin()) {
      s_.estimator.theta_R_dot_lower = NAN;
      valid = false;
    } else {
      s_.estimator.theta_R_dot_lower = it->first;
      ss0_ = it->second;
    }
  }

  if (valid) {
    alpha_ = (rate - s_.estimator.theta_R_dot_lower) /
             (s_.estimator.theta_R_dot_upper - s_.estimator.theta_R_dot_lower);
  }
  return valid;
}

float GainSchedule::rate() {
  return rate_;
}
void GainSchedule::set_state(const vector_t<state_size>& state) {
  state_ = state;
}

void GainSchedule::state_estimate(float torque_prev)
{
  if (state_estimate_time_ == s_.loop_count)
    return;
  state_estimate_time_ = s_.loop.count;
  vector_t<input_size> input {torque_prev, s_.encoder.steer, MPU6050::phi_dot(s_)};
  auto state0 = std::get<estimator>(ss0_).update(state_, input);
  auto state1 = std::get<estimator>(ss1_).update(state_, input);
  state_ = alpha_ * (state1 - state0) + state0;
  s_.estimator.phi = state_(0, 0);
  s_.estimator.delta = state_(1, 0);
  s_.estimator.phi_dot = state_(2, 0);
  s_.estimator.delta_dot = state_(3, 0);
}

const float GainSchedule::lqr_output() {
  const float t0 = std::get<lqr>(ss0_).update(state_)(0, 0);
  const float t1 = std::get<lqr>(ss1_).update(state_)(0, 0);
  return alpha_ * (t1 - t0) + t0;
}

const float GainSchedule::pi_output() {
  vector_t<output_size> x {s.pi.x};
  vector_t<output_size> e {s.pi.e};
  const float t0 = std::get<pi>(ss0_).update(x, e)(0, 0);
  const float t1 = std::get<pi>(ss1_).update(x, e)(0, 0);
  return alpha_ * (t1 - t0) + t0;
}

float GainSchedule::update_estimate_output(float torque_prev) {
  set_sample_states();
  state_estimate(torque_prev);
  return lqr_output() + pi_output();
}

} // namespace control
