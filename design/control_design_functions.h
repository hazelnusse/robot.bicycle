#ifndef CONTROL_DESIGN_FUNCTIONS_H
#define CONTROL_DESIGN_FUNCTIONS_H

#include <Eigen/Dense>
#include "bicycle/bicycle.h"

struct model_data {
  double theta_R_dot;
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::MatrixXd C_yaw_rate;

  Eigen::MatrixXcd plant_evals_c;
  Eigen::MatrixXcd plant_evecs_c;

  // Discrete time versions of A and B
  Eigen::MatrixXd A_d;
  Eigen::MatrixXd B_d;
  Eigen::MatrixXcd plant_evals_d;
  Eigen::MatrixXcd plant_evecs_d;

  // LQR Optimal state feedback control gain
  Eigen::MatrixXd K_lqr;        // Computed with care()
  Eigen::MatrixXcd evals_lqr;   // Eigenvalues of (A + B * K_lqr)
  Eigen::MatrixXcd evecs_lqr;   // Eigenvectors of (A + B * K_lqr)
  double tau_min;               // Time constant of fastest eigenvalue of (A + B * K_lqr)
  int index_fastest_eval;       // Index into eigenalue array of fastest eigenvalue

  Eigen::MatrixXd K_lqr_d;      // Computed with dare()
  Eigen::MatrixXcd evals_lqr_d; // Eigenvalues of (A_d + B_d * K_lqr_d)
  Eigen::MatrixXcd evecs_lqr_d; // Eigenvectors of (A_d + B_d * K_lqr_d)
  double tau_min_d;             // Time constant of fastest eigenvalue of (A_d + B_d * K_lqr_d)
  int index_fastest_eval_d;       // Index into eigenalue array of fastest eigenvalue

  // Unstable subspace method
  Eigen::MatrixXd K_u;
  Eigen::MatrixXd K_u_d;
  Eigen::MatrixXcd evals_u;   // Eigenvalues of (A + B * K_u)
  Eigen::MatrixXcd evecs_u;   // Eigenvectors of (A + B * K_u)
  Eigen::MatrixXcd evals_u_d;   // Eigenvalues of (A_d + B_d * K_ud)
  Eigen::MatrixXcd evecs_u_d;   // Eigenvectors of (A_d + B_d * K_ud)

  // Reduced order observer gains
  Eigen::MatrixXd K_obs;
  Eigen::MatrixXd A_obs;
  Eigen::MatrixXd B_obs;
  
  // Discrete time version of observer state equation matrices
  Eigen::MatrixXd A_obs_d;
  Eigen::MatrixXd B_obs_d;

  // Continuous time Kalman filter
  Eigen::MatrixXd K_kalman;
  Eigen::MatrixXd A_kalman;
  Eigen::MatrixXd B_kalman;

  // Discrete time version of Kalman filter
  Eigen::MatrixXd A_kalman_d;
  Eigen::MatrixXd B_kalman_d;

  // PI yaw rate controller in parallel form is:
  // Kp + Ki / s
  double Kp, Ki;
  // Discrete time yaw rate PI controller gains in pform
  double Kp_d, Ki_d;

  bool operator<(const model_data & other) const {
      return theta_R_dot < other.theta_R_dot;
  }
};

struct design_parameters {
  int N;                         // # of speeds to make calculations at
  double Ts;                     // sample time
  double lowest_speed;           // m/s
  double highest_speed;          // m/s
  Eigen::MatrixXd Q;             // LQR Q weighting
  Eigen::MatrixXd R;             // LQR R weighting
  double pole_placement_factor;  // Observer pole locations (for pole placement design)
  Eigen::MatrixXd W;             // Process noise covariance (for LQG observer design)
  Eigen::MatrixXd V;             // Measurement noise covariance (for LQG observer design)
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

void compute_pi_gains(const design_parameters & params,
                      bicycle::Bicycle & bike,
                      model_data & data);

std::vector<model_data> design_controller(const design_parameters & params,
                                          bicycle::Bicycle & bike);

#endif // CONTROL_DESIGN_FUNCTIONS_H

