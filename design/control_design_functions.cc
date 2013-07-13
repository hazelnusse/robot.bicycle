#include <algorithm>
#include <cmath>
#include <complex>
#include <stdexcept>
#include <iostream>
#include <tuple>

#include <control/control.h>

#include "robot_bicycle_parameters.h"
#include "control_design_functions.h"
#include "matlab_interface.h"

MatlabInterface matlab;

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

    matlab.put_Matrix(m.A, "A");
    matlab.put_Matrix(m.B, "B");
    Eigen::MatrixXd Ts_matrix(1, 1);  Ts_matrix << params.Ts;
    matlab.put_Matrix(Ts_matrix, "Ts");
    matlab.eval("sys_c = ss(A, B, eye(4), zeros(4, 1));");
    matlab.eval("sys_d = c2d(sys_c, Ts, 'tustin');");
    matlab.eval("A_d = sys_d.A;");
    matlab.eval("B_d = sys_d.B;");
    m.A_d = matlab.get_Matrix("A_d");
    m.B_d = matlab.get_Matrix("B_d");

    Eigen::EigenSolver<Eigen::MatrixXd> plant_c(m.A);
    m.plant_evals_c = plant_c.eigenvalues();
    m.plant_evecs_c = plant_c.eigenvectors();
    Eigen::EigenSolver<Eigen::MatrixXd> plant_d(m.A);
    m.plant_evals_d = plant_d.eigenvalues();
    m.plant_evecs_d = plant_d.eigenvectors();
}

void compute_lqr_gains(const design_parameters & params,
                       bicycle::Bicycle & bike,
                       model_data & data)
{
  // Using lqr()
  matlab.put_Matrix(params.Q, "Q");
  matlab.put_Matrix(params.R, "R");
  matlab.eval("[K_lqr, S, e] = lqr(sys_c, Q, R); K_lqr = -K_lqr;");
  data.K_lqr = matlab.get_Matrix("K_lqr");
  Eigen::EigenSolver<Eigen::MatrixXd> es(data.A + data.B * data.K_lqr);
  data.evals_lqr = es.eigenvalues();
  data.evecs_lqr = es.eigenvectors();
  std::cout << "Uncontrolled plant evals (continuous): " << data.plant_evals_c.transpose() << std::endl;
  std::cout << "K_lqr =\n" << data.K_lqr << std::endl;
  std::cout << "Eigenvalues of A + B*K_lqr\n" << data.evals_lqr.transpose() << std::endl;
  //std::cout << "Eigenvectors of A + B*K_lqr\n" << data.evecs_lqr << std::endl;

  Eigen::ArrayXd tau(4);
  data.index_fastest_eval = 0;
  for (int i = 0; i < 4; ++i) {
    double zeta = -std::cos(std::arg(data.evals_lqr(i)));
    double wn = std::abs(data.evals_lqr(i));
    tau(i) = 1/zeta/wn;

    if (data.evals_lqr(i).real() < data.evals_lqr(data.index_fastest_eval).real())
      data.index_fastest_eval = i;
  }
  data.tau_min = tau.minCoeff();
  //std::cout << "time constant = " << data.tau_min << " seconds" << std::endl;
//  std::cout << "characteristic frequency = " << 1.0 / data.tau_min << " rad / s"
//            << " = " << 1.0/(2.0*M_PI*data.tau_min) << " Hz" << std::endl;
  //std::cout << "Fastest eigenvalue index = " << data.index_fastest_eval << std::endl;

  // Using dlqr()
  matlab.eval("[K_lqr_d, S_d, e_d] = dlqr(sys_d.A, sys_d.B, Q, R); K_lqr_d = -K_lqr_d;");
  data.K_lqr_d = matlab.get_Matrix("K_lqr_d");
  Eigen::EigenSolver<Eigen::MatrixXd> es_d(data.A_d + data.B_d * data.K_lqr_d);
  data.evals_lqr_d = es_d.eigenvalues();
  data.evecs_lqr_d = es_d.eigenvectors();
  //std::cout << "Discrete LQR gain:\n" << data.K_lqr_d << std::endl;
  //std::cout << "Eigenvalues of A_d + B_d*K_lqr_d\n" << data.evals_lqr_d.transpose() << std::endl;
  //std::cout << "Eigenvectors of A_d + B_d*K_lqr_d\n" << data.evecs_lqr_d << std::endl;

  Eigen::ArrayXd tau_d(4);
  data.index_fastest_eval_d = 0;
  for (int i = 0; i < 4; ++i) {
    double zeta = -std::cos(std::arg(std::log(data.evals_lqr_d(i))));
    double wn = std::abs(std::log(data.evals_lqr_d(i))) / params.Ts;
    tau_d(i) = 1/zeta/wn;

    if (std::abs(data.evals_lqr_d(i)) < std::abs(data.evals_lqr_d(data.index_fastest_eval_d)))
      data.index_fastest_eval_d = i;
  }
  data.tau_min_d = tau_d.minCoeff();
  //std::cout << "time constant = " << data.tau_min_d << " seconds" << std::endl;
  //std::cout << "characteristic frequency = " << 1.0 / data.tau_min_d << " rad / s"
  //          << " = " << 1.0/(2.0*M_PI*data.tau_min_d) << " Hz" << std::endl;
  //std::cout << "Fastest eigenvalue index = " << data.index_fastest_eval_d << std::endl;

  // Control only the unstable subspace
  matlab.eval("[U, T] = schur(A); [US, TS] = ordschur(U, T, 'lhp'); E = ordeig(T); Er = real(E); Ei = imag(E);");
  Eigen::MatrixXd U = matlab.get_Matrix("US");
  Eigen::MatrixXd T = matlab.get_Matrix("TS");
  Eigen::MatrixXd Er = matlab.get_Matrix("Er");
  Eigen::MatrixXd Ei = matlab.get_Matrix("Ei");
  //std::cout << "Ordered eigenvalues (real): " << Er.transpose() << std::endl;
  //std::cout << "Ordered eigenvalues (imag): " << Ei.transpose() << std::endl;
  int p = 0;
  for (int i = 0; i < Er.size(); ++i) if (Er(i, 0) > 0) ++p;
  if (p) {  // we have an unstable subspace
    Eigen::MatrixXd Tu = T.bottomRightCorner(p, p);
    Eigen::MatrixXd Bu = (U.transpose() * data.B).bottomRows(p);
    Eigen::MatrixXd Qu = (U.transpose() * params.Q * U).bottomRightCorner(p, p);
    Eigen::MatrixXd zu_from_x = U.transpose().bottomRows(p);
    Eigen::MatrixXd dim_unstable(1, 1); dim_unstable << p;
    matlab.put_Matrix(Tu, "Tu");
    matlab.put_Matrix(Bu, "Bu");
    matlab.put_Matrix(Qu, "Qu");
    matlab.put_Matrix(dim_unstable, "p");
    // matlab.eval("sys_unstable_subspace = ss(Tu, Bu, eye(p), zeros(p, 1)); [K_u, S_u, e_u] = lqr(sys_unstable_subspace, Qu, R); K_u = -K_u;");
    matlab.eval("evals_u = eig(Tu);  evals_u = evals_u - 2*real(evals_u); [K_u, prec, message] = place(Tu, Bu, evals_u); K_u = -K_u;");
    // matlab.eval("sys_unstable_subspace_d = c2d(sys_unstable_subspace, Ts, 'tustin'); [K_u_d, prec, p] = dlqr(sys_unstable_subspace_d.A, sys_unstable_subspace_d.B, Qu, R); K_u_d = -K_u_d;");
    data.K_u = matlab.get_Matrix("K_u") * zu_from_x;
    // data.K_u_d = matlab.get_Matrix("K_u_d") * zu_from_x;
  } else {
    data.K_u = Eigen::MatrixXd::Zero(1, 4);
    // data.K_u_d = Eigen::MatrixXd::Zero(1, 4);
  }
  std::cout << "K_u = " << data.K_u << std::endl;
  Eigen::EigenSolver<Eigen::MatrixXd> es2(data.A + data.B * data.K_u);
  data.evals_u = es2.eigenvalues();
  data.evecs_u = es2.eigenvectors();
  //Eigen::EigenSolver<Eigen::MatrixXd> es3(data.A_d + data.B_d * data.K_u_d);
  //data.evals_u_d = es3.eigenvalues();
  //data.evecs_u_d = es3.eigenvectors();
  std::cout << "Eigenvalues of A + B*K_u\n" << data.evals_u.transpose() << std::endl;
//  std::cout << "K_u_d = " << data.K_lqr_unstable << std::endl;
//  std::cout << "Original plant evals (discrete): " << data.plant_evals_d.transpose() << std::endl;
//  std::cout << "Controlled plant evals (discrete): " << (data.A_d + data.B_d * data.K_lqr_unstable_d).eigenvalues().transpose() << std::endl;
}

void compute_observer_gains(const design_parameters & params,
                            bicycle::Bicycle & bike,
                            model_data & data)
{
  Eigen::MatrixXd C_meas(2, 4);
  C_meas << 0, 1, 0, 0,
           0, 0, 1, 0;
  matlab.put_Matrix(C_meas, "C_meas");
  Eigen::MatrixXd poles(4, 1);
  poles(0, 0) = data.evals_lqr(data.index_fastest_eval).real() * params.pole_placement_factor;
  poles(1, 0) = poles(0, 0) - 0.2;
  poles(2, 0) = poles(0, 0) - 0.4;
  poles(3, 0) = poles(0, 0) - 0.6;
  matlab.put_Matrix(poles, "poles");
  matlab.eval("[K_obs, prec, message] = place(A', C_meas', poles); K_obs = K_obs';");
  data.K_obs = matlab.get_Matrix("K_obs");
  std::cout << "K_obs = \n" << data.K_obs << std::endl;
  Eigen::EigenSolver<Eigen::MatrixXd> es(data.A - data.K_obs * C_meas);
  std::cout << "Observer CL evals = \n" << es.eigenvalues().transpose() << std::endl;
  matlab.eval("B_obs = [K_obs, B]; sys_obs_c = ss(A - K_obs * C_meas, B_obs, eye(4), zeros(4, 3));"
              "A_obs = sys_obs_c.A;"
              "sys_obs_d = c2d(sys_obs_c, Ts, 'tustin'); A_obs_d = sys_obs_d.A; B_obs_d = sys_obs_d.B;");
  // matlab.eval("observer_tf = zpk(sys_obs_c); observer_tf.DisplayFormat='frequency'; observer_tf");
  data.A_obs = matlab.get_Matrix("A_obs");
  data.B_obs = matlab.get_Matrix("B_obs");
  data.A_obs_d = matlab.get_Matrix("A_obs_d");
  data.B_obs_d = matlab.get_Matrix("B_obs_d");
  //std::cout << "Observer A matrix:\n" << data.A_obs_d << std::endl;
  //std::cout << "Observer B matrix:\n" << data.B_obs_d << std::endl;
  // std::cout << "Observer A_d evals:\n" << data.A_obs_d.eigenvalues().transpose() << std::endl;

  // LQG Design
  matlab.put_Matrix(params.W, "W");
  matlab.put_Matrix(params.V, "V");
  matlab.eval("sys_plant = ss(A, [B, eye(4)], C_meas, zeros(2, 5)); [kest, L, P] = kalman(sys_plant, W, V);");
  matlab.eval("A_kalman = A - L * C_meas; B_kalman = [L, B]; sys_kalman = ss(A_kalman, B_kalman, eye(4), zeros(4, 3)); sys_kalman_d = c2d(sys_kalman, Ts, 'tustin'); A_kalman_d = sys_kalman_d.A; B_kalman_d = sys_kalman_d.B;");
  //matlab.eval("tf_kalman = zpk(sys_kalman); tf_kalman.DisplayFormat = 'frequency'; tf_kalman");
  data.K_kalman = matlab.get_Matrix("L");
  data.A_kalman = matlab.get_Matrix("A_kalman");
  data.B_kalman = matlab.get_Matrix("B_kalman");
  data.A_kalman_d = matlab.get_Matrix("A_kalman_d");
  data.B_kalman_d = matlab.get_Matrix("B_kalman_d");
  std::cout << "Kalman filter evals (continuous):\n" << data.A_kalman.eigenvalues().transpose() << std::endl;
  //std::cout << "Kalman filter evals (discrete):\n" << data.A_kalman_d.eigenvalues() << std::endl;
}


void compute_pi_gains(const design_parameters & params,
                      bicycle::Bicycle & bike,
                      model_data & data)
{
  std::cout << "C_yaw_rate = " << data.C_yaw_rate << std::endl;
  matlab.put_Matrix(data.C_yaw_rate, "C_yaw_rate");
  matlab.eval("A_lqrobs_cl = [A, B*K_lqr; K_obs*C_meas, A_obs + B*K_lqr];"
              "B_lqrobs_cl = [B; B];"
              "sys_lqrobs_cl = ss(A_lqrobs_cl, B_lqrobs_cl, [C_yaw_rate, zeros(1, 4)], 0);"
              "[sys_pi, info] = pidtune(sys_lqrobs_cl, 'pi', 0.1*2*pi)");
  matlab.eval("sys_pi_d = c2d(sys_pi, Ts)");
  matlab.eval("Kp = sys_pi.Kp;"
              "Ki = sys_pi.Ki;"
              "Kp_d = sys_pi_d.Kp;"
              "Ki_d = sys_pi_d.Ki;"
              "speed_array(i) = v_i;"
              "pi_array(:, i) = [Kp; Ki];");
  data.Kp = matlab.get_Matrix("Kp")(0, 0);
  data.Ki = matlab.get_Matrix("Ki")(0, 0);
  data.Kp_d = matlab.get_Matrix("Kp_d")(0, 0);
  data.Ki_d = matlab.get_Matrix("Ki_d")(0, 0);
}

std::vector<model_data> design_controller(const design_parameters & params,
                                          bicycle::Bicycle & bike)
{
  std::vector<model_data> bicycle_speed_data;
  bicycle_speed_data.reserve(params.N);              // reserve room for N model_data's

  Eigen::MatrixXd N(1, 1); N << params.N;
  matlab.put_Matrix(N, "N");
  matlab.eval("pi_array = zeros(2, N);"
              "speed_array = zeros(1, N);"
              "i = 1;");
  // Speeds used for calculations
  Eigen::ArrayXd v = Eigen::ArrayXd::LinSpaced(params.N,
                                               std::log(params.lowest_speed),
                                               std::log(params.highest_speed)).exp();
  for (int i = 0; i != v.size(); ++i) {
    model_data data;
    data.theta_R_dot = -v[i] / (bicycle::rear.R + bicycle::rear.r);
    std::cout << "Speed = " << v[i] << std::endl;
    Eigen::MatrixXd speed(1, 1); speed << v[i]; matlab.put_Matrix(speed, "v_i");
    compute_state_space_matrices(params, bike, data);
    compute_lqr_gains(params, bike, data);
    compute_observer_gains(params, bike, data); //  observer_to_controller_ratio, delta_to_delta_dot_ratio, k0);
    compute_pi_gains(params, bike, data);

    std::cout << "\n";
    bicycle_speed_data.push_back(data);
    matlab.eval("i = i + 1;");
  }
//  matlab.eval("plot(speed_array, pi_array)");
//  fgetc(stdin);
  return bicycle_speed_data;
}
