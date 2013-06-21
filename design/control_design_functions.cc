#include <algorithm>
#include <cmath>
#include <complex>
#include <stdexcept>
#include <iostream>
#include <tuple>

#include <control/control.h>

#include "robot_bicycle_parameters.h"
#include "control_design_functions.h"


Eigen::IOFormat matlabfmt(Eigen::FullPrecision, 0, ", ", ";\n", "", "", "[", "]");
Eigen::IOFormat printfmt(Eigen::FullPrecision, 0, ", ", ", ", "", "", "{{", "}}");

void compute_state_space_matrices(const design_parameters & params,
                                  bicycle::Bicycle & bike,
                                  model_data & data)
{
    model_data & m = data;
    bike.set_state(12, m.theta_R_dot);
    bike.solve_velocity_constraints_and_set_state();
    Eigen::MatrixXd mm = bike.mass_matrix_full().topLeftCorner(14, 14);
    Eigen::MatrixXd aa = bike.independent_state_matrix().topRows(14);
    Eigen::MatrixXd bb = Eigen::MatrixXd::Zero(20, 1);
    bb.bottomRows(9) = bike.input_matrix().col(20);

    Eigen::JacobiSVD<Eigen::MatrixXd> mm_qr(mm, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd A_full = mm_qr.solve(aa);
    Eigen::MatrixXd B_full = mm_qr.solve(bb.topRows(14));
    m.A = Eigen::MatrixXd::Zero(4, 4);
    m.A(0, 2) = m.A(1, 3) = 1.0;
    m.A.row(2) << A_full(9, 1), A_full(9, 2), A_full(9, 7), A_full(9, 8);
    m.A.row(3) << A_full(11, 1), A_full(11, 2), A_full(11, 7), A_full(11, 8);
    m.B = Eigen::MatrixXd::Zero(4, 1);
    m.B(2, 0) = B_full(9);
    m.B(3, 0) = B_full(11);
    m.C_yaw_rate = Eigen::MatrixXd::Zero(1, 4);
    m.C_yaw_rate << A_full(0, 1), A_full(0, 2), A_full(0, 7), A_full(0, 8);

    std::tie(m.A_d, m.B_d) = control::continuous_to_discrete(m.A, m.B, params.Ts);
}

void compute_lqr_gains(const design_parameters & params,
                       bicycle::Bicycle & bike,
                       model_data & data)
{
  // Using care()
  Eigen::MatrixXd G = (data.B * data.B.transpose().eval()) / params.R(0, 0);
  Eigen::MatrixXd X = control::care(data.A, G, params.Q);
  Eigen::MatrixXd residual = data.A.transpose() * X + X * data.A - X * G * X + params.Q;
  if (residual.norm() > 5e-6) {   // Matlab does better... around 1e-10 not sure why
    std::cout << "A = " << data.A.format(matlabfmt) << std::endl;
    std::cout << "B = " << data.B.format(matlabfmt) << std::endl;
    std::cout << "Q = " << params.Q.format(matlabfmt) << std::endl;
    std::cout << "X = " << X.format(matlabfmt) << std::endl;
    std::cout << "residual = " << residual.format(matlabfmt) << std::endl;
    std::cout << "residual Frobenious norm = " << residual.norm() << std::endl;
    throw std::runtime_error("Ricatti equation not sastisfied.");
  }
  data.K_lqr = -data.B.transpose() * X / params.R(0, 0);
  std::cout << "continuous LQR gain:\n" << data.K_lqr << std::endl;
  Eigen::EigenSolver<Eigen::MatrixXd> es(data.A + data.B * data.K_lqr);
  data.evals_lqr = es.eigenvalues();
  data.evecs_lqr = es.eigenvectors();
  std::cout << "Eigenvalues of A + B*K\n" << data.evals_lqr.transpose() << std::endl;
  std::cout << "Eigenvectors of A + B*K\n" << data.evecs_lqr << std::endl;
  if ((data.evals_lqr.array().real() > 0).any()) {
    std::cout << data.evals_lqr_d << std::endl;
    throw std::runtime_error("Eigenvalues are in right half plane.");
  }

  Eigen::ArrayXd tau(4);
  for (int i = 0; i < 4; ++i) {
    double zeta = -std::cos(std::arg(data.evals_lqr(i)));
    double wn = std::abs(data.evals_lqr(i));
    tau(i) = 1/zeta/wn;
  }
  data.tau_min = tau.minCoeff();
  std::cout << "time constant = " << data.tau_min << " seconds" << std::endl;
  std::cout << "characteristic frequency = " << 1.0 / data.tau_min << " rad / s"
            << " = " << 1.0/(2.0*M_PI*data.tau_min) << " Hz" << std::endl;

  // Using dare()
  Eigen::MatrixXd X_d = control::dare(data.A_d, data.B_d, params.R, params.Q);
  Eigen::MatrixXd ric_d_middle_term = params.R + data.B_d.transpose() * X_d * data.B_d;
  Eigen::JacobiSVD<Eigen::MatrixXd> ric_d_svd(ric_d_middle_term, Eigen::ComputeThinU | Eigen::ComputeThinV);

  Eigen::MatrixXd residual_d = data.A_d.transpose()* X_d * data.A_d - X_d
    - data.A_d.transpose() * X_d * data.B_d * ric_d_svd.solve(data.B_d.transpose() * X_d * data.A_d)
    + params.Q;
  if (residual_d.norm() > 1e-1) {   // Matlab does way better, around 1e-10 not sure why
    std::cout << "A_d = " << data.A_d.format(matlabfmt) << std::endl;
    std::cout << "B_d = " << data.B_d.format(matlabfmt) << std::endl;
    std::cout << "Q = " << params.Q.format(matlabfmt) << std::endl;
    std::cout << "X_d = " << X_d.format(matlabfmt) << std::endl;
    std::cout << "residual_d = " << residual_d.format(matlabfmt) << std::endl;
    std::cout << "residual Frobenious norm = " << residual_d.norm() << std::endl;
    throw std::runtime_error("Ricatti equation not sastisfied.");
  }
  Eigen::MatrixXd A_lqr_d = (params.R + data.B_d.transpose() * X_d * data.B_d);
  Eigen::MatrixXd B_lqr_d = data.B_d.transpose() * X_d * data.A_d;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_lqr_d(A_lqr_d, Eigen::ComputeThinU | Eigen::ComputeThinV);
  data.K_lqr_d = -svd_lqr_d.solve(B_lqr_d);
  //std::cout << "discrete LQR gain: " << data.K_lqr_d << std::endl;
  Eigen::EigenSolver<Eigen::MatrixXd> es_d(data.A_d + data.B_d * data.K_lqr_d);
  data.evals_lqr_d = es_d.eigenvalues();
  data.evecs_lqr_d = es_d.eigenvectors();
  //std::cout << "Discrete controller eigenvalues " << data.evals_lqr_d << std::endl;
  if ((data.evals_lqr_d.array().abs() > 1).any()) {
    std::cout << data.evals_lqr_d << std::endl;
    throw std::runtime_error("Eigenvalues are outside unit circle");
  }
  Eigen::ArrayXd tau_d(4);
  for (int i = 0; i < 4; ++i) {
    double zeta_d = -std::cos(std::arg(std::log(data.evals_lqr_d(i))));
    double wn_d = std::abs(std::log(data.evals_lqr_d(i))) / params.Ts;
    tau_d(i) = 1/zeta_d/wn_d;
  }
  data.tau_min_d = tau_d.minCoeff();
  //std::cout << "tau_min_d = " << data.tau_min_d << std::endl;
}

void compute_observer_gains(const design_parameters & params,
                            bicycle::Bicycle & bike,
                            model_data & data)
{
// Observer coefficients:
// w: (a20*k2 + a30*k3)/k0,
// delta: a21*k2 + a31*k3 - k1*(a20*k2 + a30*k3)/k0,
// phi_dot: a22*k2 + a32*k3 + k0 - k2*(a20*k2 + a30*k3)/k0,
// delta_dot: a23*k2 + a33*k3 + k1 - k3*(a20*k2 + a30*k3)/k0]
// T_delta: b20*k2 + b30*k3
// Top row of estimator C matrix:
// [1/k0, -k1/k0, -k2/k0, -k3/k0]

  const double a20 = data.A(2, 0),
               a21 = data.A(2, 1),
               a22 = data.A(2, 2),
               a23 = data.A(2, 3);
  const double a30 = data.A(3, 0),
               a31 = data.A(3, 1),
               a32 = data.A(3, 2),
               a33 = data.A(3, 3);
  const double b20 = data.B(2, 0),
               b30 = data.B(3, 0);
               

  const double desired_eval = -1.0/(data.tau_min / params.observer_to_controller_ratio);
  // Assume k2 == k3:
  const double k2 = desired_eval * params.k0 / (a20 + a30);
  const double k3 = k2;
  // Solve for k1 to make the steer coefficient N times bigger than the steer
  // rate coefficient
  const double N = params.delta_to_delta_dot_ratio;
  const double k1 = (N*a20*k2*k3 - N*a23*params.k0*k2 + N*a30*std::pow(k3, 2) - N*a33*params.k0*k3 + a21*params.k0*k2 + a31*params.k0*k3)/(N*params.k0 + a20*k2 + a30*k3);

  data.K_obs = Eigen::MatrixXd::Zero(4, 1);
  data.K_obs(0, 0) = params.k0;
  data.K_obs(1, 0) = k1;
  data.K_obs(2, 0) = k2;
  data.K_obs(3, 0) = k3;

  data.A_obs = Eigen::MatrixXd::Zero(1, 1);
  data.A_obs(0, 0) = (a20*k2 + a30*k3)/params.k0;                            // w coefficient

  data.B_obs = Eigen::MatrixXd::Zero(1, 4);
  data.B_obs(0, 0) = a21*k2 + a31*k3 - k1*(a20*k2 + a30*k3)/params.k0;       // delta coefficient
  data.B_obs(0, 1) = a22*k2 + a32*k3 + params.k0 - k2*(a20*k2 + a30*k3)/params.k0;  // phi dot coefficent
  data.B_obs(0, 2) = a23*k2 + a33*k3 + k1 - k3*(a20*k2 + a30*k3)/params.k0;  // delta dot coefficient
  data.B_obs(0, 3) = b20*k2 + b30*k3;                                 // T_delta coefficient
  if (std::abs(data.B_obs(0, 0) - N * data.B_obs(0, 2)) > 1e-10) {    // verify ratio
    std::cout << "B(0, 0) = " << data.B_obs(0, 0) << std::endl;
    std::cout << "N * B(0, 2) = " << N * data.B_obs(0, 2) << std::endl;
    throw std::runtime_error("Ratio calculation failed.");
  }

  data.C_obs = Eigen::MatrixXd::Zero(1, 1);
  data.C_obs(0, 0) = 1 / params.k0;

  data.D_obs = Eigen::MatrixXd::Zero(1, 4);
  data.D_obs(0, 0) = -k1 / params.k0;
  data.D_obs(0, 1) = -k2 / params.k0;
  data.D_obs(0, 2) = -k3 / params.k0;
  data.D_obs(0, 3) = 0;

  std::cout << "observer time constant = " << -1/data.A_obs(0, 0) << " seconds\n";
  std::cout << "observer characteristic frequency = " << -data.A_obs(0, 0) << " rad / s"
            << " = " << -data.A_obs(0, 0)/(2.0*M_PI) << " Hz" << std::endl;
  std::cout << "A_obs = " << data.A_obs.format(matlabfmt) << std::endl;
  std::cout << "B_obs = " << data.B_obs.format(matlabfmt) << std::endl;
  std::cout << "C_obs = " << data.C_obs.format(matlabfmt) << std::endl;
  std::cout << "D_obs = " << data.D_obs.format(matlabfmt) << std::endl;
}

std::vector<model_data> design_controller(const design_parameters & params,
                                          bicycle::Bicycle & bike)
{
//  std::vector<model_data> result;
//  result.reserve(params.N);
//  return result;
//
//
//  const int N = 101;                    // # of speeds to make calculations at
//  const double lowest_speed = 0.5;      // m/s
//  const double highest_speed = 10.0;    // m/s
  std::vector<model_data> bicycle_speed_data;
  bicycle_speed_data.reserve(params.N);              // reserve room for N model_data's
//
  // Speeds used for calculations
  Eigen::ArrayXd v = Eigen::ArrayXd::LinSpaced(params.N,
                                               std::log(params.lowest_speed),
                                               std::log(params.highest_speed)).exp();
//
//  // LQR design parameters
//  constexpr double pi = M_PI;
//  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, 4);
//  Q(0, 0) = std::pow(30*pi/180, -2.0);
//  Q(1, 1) = std::pow(30*pi/180, -2.0);
//  Q(2, 2) = std::pow(200*pi/180, -2.0);
//  Q(3, 3) = std::pow(200*pi/180, -2.0);
//  Eigen::MatrixXd R(1, 1);
//  R(0, 0) = std::pow(6.0*.75, -2.0);
//
//  const double k0 = 0.25;
//  const double observer_to_controller_ratio = 5.0;
//  const double delta_to_delta_dot_ratio = 2.0;
//
  for (int i = 0; i != v.size(); ++i) {
    model_data data;
    data.theta_R_dot = -v[i] / (bicycle::rear.R + bicycle::rear.r);
    std::cout << "Speed = " << v[i] << std::endl;
    compute_state_space_matrices(params, bike, data);
    compute_lqr_gains(params, bike, data);
    compute_observer_gains(params, bike, data); //  observer_to_controller_ratio, delta_to_delta_dot_ratio, k0);

    std::cout << "\n";
    bicycle_speed_data.push_back(data);
  }
  return bicycle_speed_data;
}
