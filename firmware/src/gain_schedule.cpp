#include <algorithm>
#include <cmath>

#include "gain_schedule.h"
#include "MPU6050.h"

namespace control {

vector_t<observer_state_size> StateEstimator::update(const vector_t<observer_state_size>& x,
                                            const vector_t<observer_input_size>& u) const
{
  return A * x + B * u;
}

vector_t<observer_output_size> StateEstimator::output(const vector_t<observer_state_size>& x,
                                            const vector_t<observer_input_size>& u) const
{
  return C * x + D * u;
}

vector_t<plant_model_input_size> LQRController::update(const vector_t<plant_model_state_size>& x) const
{
  return C * x;
}

//vector_t<output_size> PIController::update(const vector_t<output_size>& x,
//                                           const vector_t<output_size>& e) const
//{
//  return Kp * x + Ki * e;
//}

bool rt_controller_t::operator<(const rt_controller_t& rhs) const
{
  return rate < rhs.rate;
}

GainSchedule::GainSchedule()
  : state_{{}}, pi_control_enabled_{false},
    derivative_filter_{0, 50*2*constants::pi,
                       50*2*constants::pi, constants::loop_period_s}
{

}

float GainSchedule::rate() const
{
  return rate_;
}

bool GainSchedule::set_sample(Sample& s)
{
  s_ = &s;
  return set_rate(s.encoder.rear_wheel_rate);
}

void GainSchedule::set_state(const vector_t<plant_model_state_size>& state)
{
  state_ = state;
}

bool GainSchedule::set_rate(float rate)
{
  bool valid = true;
  rate_ = rate;

  r.rate = rate;
  auto it = std::upper_bound(schedule_.begin(), schedule_.end(), r);
  if (it == schedule_.begin() || it == schedule_.end()) {
    valid = s_->has_estimate = false;
    if (it == schedule_.begin()) {
      s_->estimate.theta_R_dot_upper = it->rate;
      s_->estimate.theta_R_dot_lower = NAN;
    } else {
      s_->estimate.theta_R_dot_upper = NAN;
      s_->estimate.theta_R_dot_lower = (--it)->rate;
    }
  } else {
    valid = s_->has_estimate = true;
    s_->estimate.theta_R_dot_upper = it->rate;
    ss_upper_ = const_cast<controller_t*>(&(it->controller));
    s_->estimate.theta_R_dot_lower = (--it)->rate;
    ss_lower_ = const_cast<controller_t*>(&(it->controller));

    alpha_ = (rate - s_->estimate.theta_R_dot_lower) /
             (s_->estimate.theta_R_dot_upper - s_->estimate.theta_R_dot_lower);
  }

  return valid;
}

void GainSchedule::state_estimate(float torque_prev)
{
  state_estimate_time_ = s_->loop_count;
  s_->estimate.delta = s_->encoder.steer;
  s_->estimate.phi_dot = hardware::MPU6050::phi_dot(*s_);
  s_->estimate.delta_dot = derivative_filter_.output(s_->encoder.steer);
  derivative_filter_.update(s_->encoder.steer); // update for next iteration

  vector_t<observer_input_size> input {{s_->estimate.delta, s_->estimate.phi_dot,
                                        s_->estimate.delta_dot, torque_prev}};

  // estimator output and interpolation (roll angle)
  auto state_lower = ss_lower_->estimator.output(w_, input);
  auto state_upper = ss_upper_->estimator.output(w_, input);
  s_->estimate.phi = (alpha_ * (state_upper - state_lower) + state_lower)(0, 0);
  state_(0, 0) = s_->estimate.phi;
  state_(0, 1) = s_->estimate.delta;
  state_(0, 2) = s_->estimate.phi_dot;
  state_(0, 3) = s_->estimate.delta_dot;

  auto w_lower = ss_lower_->estimator.update(w_, input);
  auto w_upper = ss_upper_->estimator.update(w_, input);
  w_ = alpha_ * (w_upper - w_lower) + w_lower;
  s_->estimate.w = w_(0, 0);
  s_->estimate.has_w = true;
}

float GainSchedule::lqr_output() const
{
  const float t0 = ss_lower_->lqr.update(state_)(0, 0);
  const float t1 = ss_upper_->lqr.update(state_)(0, 0);
  return alpha_ * (t1 - t0) + t0;
}

float GainSchedule::pi_output() const
{
//  vector_t<output_size> x {{s_->yaw_rate_pi.x}};
//  vector_t<output_size> e {{s_->yaw_rate_pi.e}};
  float x = s_->yaw_rate_pi.x;
  float e = s_->yaw_rate_pi.e;
//  const float t0 = ss_lower_->pi.update(x, e)(0, 0);
//  const float t1 = ss_upper_->pi.update(x, e)(0, 0);
  const float t0 = ss_lower_->pi.update(x, e);
  const float t1 = ss_upper_->pi.update(x, e);
  return alpha_ * (t1 - t0) + t0;
}

float GainSchedule::compute_updated_torque(float torque_prev)
{
  state_estimate(torque_prev);
  return lqr_output(); //  + pi_output();
}

} // namespace control
