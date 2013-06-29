#include <algorithm>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include "bicycle/bicycle.h"
#include "control_design_functions.h"
#include "firmware_generator.h"
#include "robot_bicycle_parameters.h"

int main(int argc, char ** argv)
{
  bicycle::Bicycle rb = bicycle::robot_bicycle();

  design_parameters params;
  params.N = 101;
  params.Ts = 0.005;
  params.lowest_speed = 0.5;
  params.highest_speed = 10.0;
  // LQR design parameters
  constexpr double pi = M_PI;
  constexpr double max_lean = 2.0*pi/180.0;    // rad
  constexpr double max_steer = 5.0*pi/180.0;   // rad
  constexpr double max_lean_frequency = 1*pi;  // rad / s
  constexpr double max_steer_frequency = 20*pi;// rad / s
  constexpr double max_steer_torque = 4.00;     // N * m
  params.Q = Eigen::MatrixXd::Zero(4, 4);
  params.Q(0, 0) = std::pow(max_lean, -2.0);
  params.Q(1, 1) = std::pow(max_steer, -2.0);
  params.Q(2, 2) = std::pow(max_lean_frequency * max_lean, -2.0);
  params.Q(3, 3) = std::pow(max_steer_frequency * max_steer, -2.0);
  params.R.resize(1, 1);
  params.R << std::pow(max_steer_torque, -2.0);
  // Observer pole placement factor
  params.pole_placement_factor = 3.0;
  // Kalman design
  constexpr double lean_vel_std = .6827;      // rad / s / s
  constexpr double steer_vel_std = .6827;     // rad / s / s
  constexpr double lean_acc_std = 1;          // rad / s / s
  constexpr double steer_acc_std = 1;         // rad / s / s
  params.W.resize(4, 4);          // Process noise covariance
  params.W << std::pow(lean_vel_std, 2.0), 0.0, 0.0, 0.0,
              0.0, std::pow(steer_vel_std, 2.0), 0.0, 0.0,
              0.0, 0.0, std::pow(lean_acc_std, 2.0), 0.0,
              0.0, 0.0, 0.0, std::pow(steer_acc_std, 2.0);

  params.V.resize(2, 2);          // Measurement noise covariance
  params.V << std::pow(2*pi/20000, 2.0), 0,
              0, std::pow(0.00227631723111, 2.0);
  params.V *= 0.01;

  std::vector<model_data> md = design_controller(params, rb);
  std::sort(md.begin(), md.end());
  firmware_generator(md);
}

