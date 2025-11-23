% Main Script
% This Code is SITL (Software in the loop)
% Ver.2.0.0 (only PID)
% 2024-08-12 / Controla Project
% Made by NDH & LJC

%% Initialize Matlab
clc;
clear all;
close all;

%% Add Library directory
addpath(genpath('C:\Users\USER\Desktop\Kalman_Filter\Sat_ver2\main'));

%% ROS2 setting
ros2 = ros2node('matlab_simulation_node');
pub = ros2publisher(ros2, '/robot_state', 'geometry_msgs/PoseStamped');

%% Initialize setting
params = Params_init(); % Load Parameter struct

for superloop = 1:1

X = [517; 484; 0; 0; 0; 0]; % ( posX, posY, V_X, V_Y, rotZ, rateZ )
Master_point = [2400; 2400; 0; 0; 0; 0]; % Initialize Mothership position and orientation

Rk = params.EKF.Rm;

Q_Fx = 0;
Q_Fy = 0;
Q_Tau = 0;
Qw = diag([Q_Fx.^2, Q_Fx.^2, Q_Tau.^2]');

% Simulation setting
Main_Hz = params.System.MainLoopHz;
Sensor_VI_Hz = params.Sensor.VICON_HZ;
PID_control_Hz = Main_Hz;
OPENCV_HZ = params.Sensor.OPENCV_HZ;

sim_dt = 1 / Main_Hz; % [sec]
sim_Time = 160; % [sec]
sim_step = ceil(sim_Time / sim_dt);

% Sensor & Control interval Setting
Sensor_interval = Main_Hz / OPENCV_HZ; % sensor sampling set up
PID_control_interval = Main_Hz / PID_control_Hz;

% Time duty initialize
real_time = 0;

% Data log initialize
state_history = zeros(6, sim_step);
Masterpoint_history = zeros(6, sim_step);
temp3sigma_EKF = zeros(6, sim_step);
temp3sigma_UKF = zeros(6, sim_step);
X_raw_hist = zeros(6, sim_step);
X_predict_hist = zeros(6, sim_step);
debug_control = zeros(9, sim_step);
t = zeros(1, sim_step);
target_history = zeros(6, sim_step);
debug_cmd_delta_hist = zeros(9, sim_step);
X_measure = zeros(6, sim_step);
X_filt_hist = zeros(6, sim_step);
X_filt_EKF_hist = zeros(6, sim_step);
X_filt_UKF_hist = zeros(6, sim_step);
X_predict_EKF_hist = zeros(6, sim_step);
X_predict_UKF_hist = zeros(6, sim_step);
z_true_hist = zeros(3, sim_step);
state_filter_hist = zeros(3, sim_step);
z_measurement_hist = zeros(3, sim_step);

%% Main simulation loop
for i = 1:sim_step

    X_target = generate_commands(real_time); % Generate - X_target & tolerance

    if mod(i - 1, PID_control_interval) == 0

        U = Controller_PID_Argument(X, X_target, sim_dt, params);

    end

    [final_cmd, debug_cmd_delta] = Control_Allocator(U, params);

    % Stochastic RK4 simulation
    X = srk4(@vehicle_dynamics_Airbearing_stochastic, X, final_cmd, Qw, sim_dt, params, sim_dt);

    % data save
    state_history(:, i) = X;
    Masterpoint_history(:, i) = Master_point;
    debug_control(:, i) = final_cmd;
    t(i) = real_time;
    target_history(:, i) = X_target;

    real_time = real_time + sim_dt;
end

%% EKF와 UKF 적용을 위한 변수 초기화
X_filt_EKF = state_history(:, 1);
X_filt_UKF = state_history(:, 1);
P_ekf = params.EKF.P;
P_ukf = params.EKF.P;

%% 필터 적용 루프
for k = 1:sim_step

    if mod(k - 1, Sensor_interval) == 0

        X_true = state_history(:, k);
        U_true = debug_control(:, k);
        X_target_ekf = target_history(:, k);
        master_point_current = Masterpoint_history(:, k);

        [z_AHRS, z_AHRS_true] = measure_sensor_AHRS(X_true, master_point_current, Rk);
        [z_Aruco, z_Aruco_true] = measure_sensor_Aruco(X_true, master_point_current, Rk);

        z = [z_Aruco; z_AHRS];
        z_true = [z_Aruco_true; z_AHRS_true];

        % EKF 적용
        [X_filt_EKF, P_ekf, y_hat_EKF, X_predict_EKF] = processDataSIM_Muti_EKF(z, P_ekf, X_filt_EKF, master_point_current, U_true, params, sim_dt);

        % UKF 적용
        [X_filt_UKF, P_ukf, y_hat_UKF, X_predict_UKF] = processDataSIM_Muti_EKF_AXBU(z, P_ukf, X_filt_UKF, master_point_current, U_true, params, sim_dt);

    end

    % 결과 저장
    X_filt_EKF_hist(:, k) = X_filt_EKF;
    X_filt_UKF_hist(:, k) = X_filt_UKF;
    temp3sigma_EKF(:, k) = 3 * sqrt(diag(P_ekf));
    temp3sigma_UKF(:, k) = 3 * sqrt(diag(P_ukf));
    X_predict_EKF_hist(:, k) = X_predict_EKF;
    X_predict_UKF_hist(:, k) = X_predict_UKF;
    z_measurement_hist(:, k) = z;
    z_true_hist(:, k) = z_true;

    % 상태 필터 히스토리 저장 (예시로 EKF 사용)
    X_rel_I = master_point_current(1:2) - X_filt_EKF(1:2);
    X_rel_B = [cos(X_filt_EKF(5)), sin(X_filt_EKF(5)); -sin(X_filt_EKF(5)), cos(X_filt_EKF(5))] * X_rel_I;
    rho_k = sqrt(X_rel_B(1)^2 + X_rel_B(2)^2);
    theta_k = atan2(X_rel_B(2), X_rel_B(1));
    r_k = X_filt_EKF(6);

    % Measurement vector
    h1 = [rho_k; theta_k; r_k];

    state_filter_hist(:, k) = h1;

end

%% 결과 플로팅

figure(1);
subplot(3, 1, 1);
plot(t, state_history(1, :), 'b', t, target_history(1, :), 'r--');
hold on
plot(t, X_filt_EKF_hist(1, :), 'g--');
plot(t, X_filt_UKF_hist(1, :), 'm--');
xlabel('[sec]')
ylabel('[mm]')
title('X Position');
legend('True State', 'Target', 'EKF-Srk1 Estimate', 'EKF dist Estimate');

subplot(3, 1, 2);
plot(t, state_history(2, :), 'b', t, target_history(2, :), 'r--');
hold on
plot(t, X_filt_EKF_hist(2, :), 'g--');
plot(t, X_filt_UKF_hist(2, :), 'm--');
xlabel('[sec]')
ylabel('[mm]')
title('Y Position');
legend('True State', 'Target', 'EKF-Srk1 Estimate', 'EKF dist Estimate');

subplot(3, 1, 3);
plot(t, rad2deg(state_history(5, :)), 'b', t, rad2deg(target_history(5, :)), 'r--');
hold on
plot(t, rad2deg(X_filt_EKF_hist(5, :)), 'g--');
plot(t, rad2deg(X_filt_UKF_hist(5, :)), 'm--');
xlabel('[sec]')
ylabel('[deg]')
title('Yaw Angle (psi)');
legend('True State', 'Target', 'EKF-Srk1 Estimate', 'EKF dist Estimate');

figure(2);

% EKF의 3시그마와 에러 플롯
subplot(3, 1, 1)
plot(t, temp3sigma_EKF(1, :), 'r-', t, temp3sigma_UKF(1, :), 'k-');
hold on;
plot(t, state_history(1, :) - X_filt_EKF_hist(1, :), 'b', t, state_history(1, :) - X_filt_UKF_hist(1, :), 'g');
hold on;
plot(t, -temp3sigma_EKF(1, :), 'r-', t, -temp3sigma_UKF(1, :), 'k-');
hold on;

title('EKF Position Error in X');
xlabel('Time(sec)');
ylabel('Error [mm]');
legend('3\sigma EKF', 'Error', '-3\sigma EKF');

% EKF의 3시그마와 에러 플롯
subplot(3, 1, 2)
plot(t, temp3sigma_EKF(2, :), 'r-', t, temp3sigma_UKF(2, :), 'k-');
hold on;
plot(t, state_history(2, :) - X_filt_EKF_hist(2, :), 'b', t, state_history(2, :) - X_filt_UKF_hist(2, :), 'g');
hold on;
plot(t, -temp3sigma_EKF(2, :), 'r-', t, -temp3sigma_UKF(2, :), 'k-');
hold on;
title('EKF Position Error in X');
xlabel('Time(sec)');
ylabel('Error [mm]');
legend('3\sigma EKF', 'Error', '-3\sigma EKF');

subplot(3, 1, 3)
plot(t, rad2deg(temp3sigma_EKF(5, :)), 'r-');
hold on;
plot(t, rad2deg(state_history(5, :) - X_filt_EKF_hist(5, :)), 'b');
plot(t, -rad2deg(temp3sigma_EKF(5, :)), 'r-');
hold on;
plot(t, rad2deg(temp3sigma_UKF(5, :)), 'k-');
hold on;
plot(t, rad2deg(state_history(5, :) - X_filt_UKF_hist(5, :)), 'g');
hold on;
plot(t, -rad2deg(temp3sigma_UKF(5, :)), 'k-');
title('EKF Yaw Angle Error');
xlabel('Time(sec)');
ylabel('Error [deg]');
legend('3\sigma EKF', 'Error', '-3\sigma EKF');



%% RMSE 계산 및 출력

% EKF RMSE
errors_EKF = state_history - X_filt_EKF_hist;
RMSE_EKF = sqrt(mean(errors_EKF.^2, 2));

% UKF RMSE
errors_UKF = state_history - X_filt_UKF_hist;
RMSE_UKF = sqrt(mean(errors_UKF.^2, 2));

% RMSE 출력
state_names = {'X position', 'Y position', 'X velocity', 'Y velocity', 'Psi angle', 'Psi rate'};

fprintf('RMSE for EKF:\n');
for i = 1:length(RMSE_EKF)
    if i == 5  % Psi angle in degrees
        RMSE_deg = rad2deg(RMSE_EKF(i));
        fprintf('RMSE for %s: %f degrees\n', state_names{i}, RMSE_deg);
    else
        fprintf('RMSE for %s: %f\n', state_names{i}, RMSE_EKF(i));
    end
end

fprintf('\nRMSE for UKF:\n');
for i = 1:length(RMSE_UKF)
    if i == 5  % Psi angle in degrees
        RMSE_deg = rad2deg(RMSE_UKF(i));
        fprintf('RMSE for %s: %f degrees\n', state_names{i}, RMSE_deg);
    else
        fprintf('RMSE for %s: %f\n', state_names{i}, RMSE_UKF(i));
    end
end

end
