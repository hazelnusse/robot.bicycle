#include <cmath>

#include "gain_schedule.h"
#include "MPU6050.h"

namespace control {
vector_t<state_size> StateEstimator::update(const vector_t<state_size>& x,
                                            const vector_t<input_size>& u) const {
  return A * x + B * u;
}
vector_t<output_size> LQRController::update(const vector_t<state_size>& x) const {
  return C * x;
}
vector_t<output_size> PIController::update(const vector_t<output_size>& x,
                                           const vector_t<output_size>& e) const {
  return Kp * x + Ki * e;
}

GainSchedule::GainSchedule() : state_{{}} { };

float GainSchedule::rate() const {
  return rate_;
}

Sample& GainSchedule::sample() {
  return s_;
}

bool GainSchedule::set_sample(Sample& s) {
  s_ = s;
  return set_rate(s.encoder.rear_wheel_rate);
}

void GainSchedule::set_state(const vector_t<state_size>& state) {
  state_ = state;
}

bool GainSchedule::set_rate(float rate)
{
  bool valid = true;
  rate_ = rate;

  auto it = schedule_.upper_bound(rate);
  if (it == schedule_.begin() || it == schedule_.end()) {
    valid = false;
    if (it == schedule_.begin()) {
      s_.estimate.theta_R_dot_upper = it->first;
      s_.estimate.theta_R_dot_lower = NAN;
    } else {
      s_.estimate.theta_R_dot_upper = NAN;
      s_.estimate.theta_R_dot_lower = (--it)->first;
    }
  } else {
    valid = true;
    s_.estimate.theta_R_dot_upper = it->first;
    ss_upper_ = const_cast<ss_tuple_t*>(&(it->second));
    s_.estimate.theta_R_dot_lower = (--it)->first;
    ss_lower_ = const_cast<ss_tuple_t*>(&(it->second));

    alpha_ = (rate - s_.estimate.theta_R_dot_lower) /
             (s_.estimate.theta_R_dot_upper - s_.estimate.theta_R_dot_lower);
  }

  return valid;
}

void GainSchedule::state_estimate(float torque_prev)
{
  if (state_estimate_time_ == s_.loop_count)
    return;
  state_estimate_time_ = s_.loop_count;
  vector_t<input_size> input {{torque_prev, s_.encoder.steer,
                               hardware::MPU6050::phi_dot(s_)}};
  auto state_lower = ss_lower_->estimator.update(state_, input);
  auto state_upper = ss_upper_->estimator.update(state_, input);
  state_ = alpha_ * (state_upper - state_lower) + state_lower;
  s_.estimate.phi = state_(0, 0);
  s_.estimate.delta = state_(1, 0);
  s_.estimate.phi_dot = state_(2, 0);
  s_.estimate.delta_dot = state_(3, 0);
}

float GainSchedule::lqr_output() const {
  const float t0 = ss_lower_->lqr.update(state_)(0, 0);
  const float t1 = ss_upper_->lqr.update(state_)(0, 0);
  return alpha_ * (t1 - t0) + t0;
}

float GainSchedule::pi_output() const {
  vector_t<output_size> x {{s_.yaw_rate_pi.x}};
  vector_t<output_size> e {{s_.yaw_rate_pi.e}};
  const float t0 = ss_lower_->pi.update(x, e)(0, 0);
  const float t1 = ss_upper_->pi.update(x, e)(0, 0);
  return alpha_ * (t1 - t0) + t0;
}

float GainSchedule::update_estimate_output(float torque_prev) {
  state_estimate(torque_prev);
  return lqr_output() + pi_output();
}

} // namespace control
