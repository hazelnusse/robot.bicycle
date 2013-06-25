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
//  if (argc != 2) {
//    std::cout << "Filename must be supplied\n";
//    return -1;
//  }
//  std::string filename_base(argv[1]);

  bicycle::Bicycle rb = bicycle::robot_bicycle();
  // bicycle::Bicycle bike = bicycle::benchmark_bicycle();

  design_parameters params;
  params.N = 101;
  params.Ts = 0.005;
  params.lowest_speed = 0.5;
  params.highest_speed = 10.0;
  params.k0 = 0.25;
  params.observer_to_controller_ratio = 5.0;
  params.delta_to_delta_dot_ratio = 2.0;
  // LQR design parameters
  constexpr double pi = M_PI;
  params.Q = Eigen::MatrixXd::Zero(4, 4);
  params.Q(0, 0) = std::pow(30*pi/180, -2.0);
  params.Q(1, 1) = std::pow(30*pi/180, -2.0);
  params.Q(2, 2) = std::pow(200*pi/180, -2.0);
  params.Q(3, 3) = std::pow(200*pi/180, -2.0);
  params.R.resize(1, 1);
  params.R << std::pow(6.0*.75, -2.0);

  std::vector<model_data> md = design_controller(params, rb);
  std::sort(md.begin(), md.end());
  firmware_generator(md);
  //firmware_generator(md, filename_base);
}

