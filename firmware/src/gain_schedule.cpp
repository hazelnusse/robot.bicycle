#include <algorithm>
#include <cmath>

#include "gain_schedule.h"
#include "MPU6050.h"
#include "sine_table.h"

namespace control {

vector_t<observer_state_size> StateEstimator::update(const vector_t<observer_state_size>& x,
                                            const vector_t<observer_input_size>& u) const
{
  return A * x + B * u;
}

//vector_t<observer_output_size> StateEstimator::output(const vector_t<observer_state_size>& x,
//                                            const vector_t<observer_input_size>& u) const
//{
//  return C * x + D * u;
//}

vector_t<plant_model_input_size> LQRController::update(const vector_t<plant_model_state_size>& x) const
{
  return C * x;
}

bool rt_controller_t::operator<(const rt_controller_t& rhs) const
{
  return rate < rhs.rate;
}

GainSchedule::GainSchedule()
  : integrator_state_{0.0f}, state_{{}}
{
  state_(0, 0) = 0.0f;
  state_(0, 1) = 0.0f;
  state_(0, 2) = 0.0f;
  state_(0, 3) = 0.0f;
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

void GainSchedule::set_state(float lean, float steer, float lean_rate, float steer_rate)
{
  state_ (0, 0) = lean;
  state_ (0, 1) = steer;
  state_ (0, 2) = lean_rate;
  state_ (0, 3) = steer_rate;
}

void GainSchedule::state_estimate(float torque_prev)
{
  state_estimate_time_ = s_->loop_count;
  vector_t<observer_input_size> input {{s_->encoder.steer, s_->mpu6050.gyroscope_x,
                                        torque_prev}};

  // update observer state
  auto state_lower = ss_lower_->estimator.update(state_, input);
  auto state_upper = ss_upper_->estimator.update(state_, input);
  state_ = alpha_ * (state_upper - state_lower) + state_lower;
  s_->estimate.lean = state_(0, 0);
  s_->estimate.steer = state_(0, 1);
  s_->estimate.lean_rate = state_(0, 2);
  s_->estimate.steer_rate = state_(0, 3);
}

float GainSchedule::lqr_output() const
{
  const float t0 = ss_lower_->lqr.update(state_)(0, 0);
  const float t1 = ss_upper_->lqr.update(state_)(0, 0);
  return alpha_ * (t1 - t0) + t0;
}

float GainSchedule::pi_output()
{
  const float yr0 = (ss_lower_->yr.C * state_)(0, 0);
  const float yr1 = (ss_upper_->yr.C * state_)(0, 0);
  s_->estimate.yaw_rate = alpha_ * (yr1 - yr0) + yr0;
  float e = s_->set_point.yaw_rate - s_->estimate.yaw_rate;
  const float t0 = ss_lower_->pi.update(e, integrator_state_);
  const float t1 = ss_upper_->pi.update(e, integrator_state_);
  integrator_state_ += e * constants::loop_period_s;
  return alpha_ * (t1 - t0) + t0;
}

float GainSchedule::compute_updated_torque(float torque_prev)
{
  state_estimate(torque_prev);
  float torque = lqr_output();
  torque += pi_output();
  return torque;
}

} // namespace control
