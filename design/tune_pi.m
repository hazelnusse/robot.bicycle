clear all;
close all;
clc;

load cl_data.mat;
simulate = 0;

num_speeds = size(A_cl_w, 1);
num_states = size(A_cl_w, 2);
num_inputs = size(B_cl_w, 3);
num_outputs = size(C_cl_w, 2);

x0 = zeros(4, 1);
x0(1) = 10 * pi / 180.0;
x0(2) = 10 * pi / 180.0;
x0(3) = 10 * pi / 180.0;
x0(4) = 10 * pi / 180.0;

D = 0;
dt = 0.005;

Kp_w = zeros(num_speeds, 1);
Ki_w = zeros(num_speeds, 1);

indices = [71, 40, 25, 15];

for i=1:num_speeds,
  A_cl = reshape(A_cl_w(i, :, :), num_states, num_states);
  B_cl = reshape(B_cl_w(i, :, :), num_states, num_inputs);
  C_cl = reshape(C_cl_w(i, :, :), num_outputs, num_states);

  sys_d = ss(A_cl, B_cl, C_cl, D, dt);

  wc = 0.5;
  [C_pi, info] = pidtune(sys_d, 'pi', wc);

  Kp_w(i) = C_pi.Kp;
  Ki_w(i) = C_pi.Ki;

  pid_ss = ss(C_pi);

  if (info.Stable ~= 1)
    disp "PI controller not stable."
  end

  if ((simulate == 1) & ((i == indices(1)) | (i == indices(2)) | (i == indices(3)) | (i == indices(4))))

      pc = series(pid_ss, sys_d);
      cl = feedback(pc, 1, -1);
      bode(cl);

      A = reshape(A_w(i, :, :), 4, 4);
      B = reshape(B_c_w(i, :, :), 4, 1);
      C_m = reshape(C_m_w(i, :, :), 2, 4);
      C_z = reshape(C_z_w(i, :, :), 1, 4);
      K_c = reshape(K_c_w(i, :, :), 1, 4);
      A_e = reshape(A_e_w(i, :, :), 4, 4);
      B_e = reshape(B_e_w(i, :, :), 4, 3);
      sim('cl_pi');
      figure;
      plot(t, u); title('Steer torque')
      figure;
      plot(t, x); title('States');
      legend('phi','delta','dphi','ddelta');
      figure;
      plot(t, e_est); title('State estimation error')
      figure;
      plot(t, e); title('Yaw rate error');
      pause;
  end
end

save('pi_gains.mat', 'Kp_w', 'Ki_w')

