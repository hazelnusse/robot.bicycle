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
  params.Q = Eigen::MatrixXd::Zero(4, 4);
  params.Q(0, 0) = std::pow(30*pi/180, -2.0); // make this 5 degrees
  params.Q(1, 1) = std::pow(30*pi/180, -2.0); // make this 10 or 15 degrees per second
  params.Q(2, 2) = std::pow(200*pi/180, -2.0);// make this equal to the (0, 0) entry times the highest frequency of that state we want in the closed loop system
  params.Q(3, 3) = std::pow(200*pi/180, -2.0);// make this equal to the (1, 1) entry times the highest frequency of that state we want in the closed loop system
  params.R.resize(1, 1);
  params.R << std::pow(.4, -2.0);
  params.pole_placement_factor = 3.0;

  std::vector<model_data> md = design_controller(params, rb);
  std::sort(md.begin(), md.end());
  firmware_generator(md);
}

