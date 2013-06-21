#ifndef CONTROL_DESIGN_FUNCTIONS_H
#define CONTROL_DESIGN_FUNCTIONS_H

#include <Eigen/Dense>
#include "bicycle/bicycle.h"

struct model_data {
  double theta_R_dot;
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::MatrixXd C_yaw_rate;

  // Discrete time versions of A and B
  Eigen::MatrixXd A_d;
  Eigen::MatrixXd B_d;

  // LQR Optimal state feedback control gain
  Eigen::MatrixXd K_lqr;        // Computed with care()
  Eigen::MatrixXcd evals_lqr;   // Eigenvalues of (A + B * K_lqr)
  Eigen::MatrixXcd evecs_lqr;   // Eigenvectors of (A + B * K_lqr)
  double tau_min;               // Time constant of fastest eigenvalue of (A + B * K_lqr)

  Eigen::MatrixXd K_lqr_d;      // Computed with dare()
  Eigen::MatrixXcd evals_lqr_d; // Eigenvalues of (A_d + B_d * K_lqr_d)
  Eigen::MatrixXcd evecs_lqr_d; // Eigenvectors of (A_d + B_d * K_lqr_d)
  double tau_min_d;             // Time constant of fastest eigenvalue of (A_d + B_d * K_lqr_d)

  // Reduced order observer gains
  Eigen::MatrixXd K_obs;
  Eigen::MatrixXd A_obs;
  Eigen::MatrixXd B_obs;
  Eigen::MatrixXd C_obs;
  Eigen::MatrixXd D_obs;

  bool operator<(const model_data & other) const {
      return theta_R_dot < other.theta_R_dot;
  }
};

struct design_parameters {
  int N;                          // # of speeds to make calculations at
  double Ts;                            // sample time
  double lowest_speed = 0.5;            // m/s
  double highest_speed = 10.0;          // m/s
  double k0;                            // K(0, 0) entry from Luenberger
  double observer_to_controller_ratio;  // how much faster to make observer pole
  double delta_to_delta_dot_ratio;      // ratio between steer and steer rate input coefficients
  Eigen::MatrixXd Q;                    // LQR Q weighting
  Eigen::MatrixXd R;                    // LQR R weighting
};

void compute_state_space_matrices(const design_parameters & params,
                                  bicycle::Bicycle & bike,
                                  model_data & data);

void compute_lqr_gains(const design_parameters & params,
                       bicycle::Bicycle & bike,
                       model_data & data);

void compute_observer_gains(const design_parameters & params,
                            bicycle::Bicycle & bike,
                            model_data & data);

std::vector<model_data> design_controller(const design_parameters & params,
                                          bicycle::Bicycle & bike);

#endif // CONTROL_DESIGN_FUNCTIONS_H

