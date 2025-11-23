% Main Script
% This Code is SITL (Software in the loop)
% Ver.2.0.0 (only PID)
% 2024-08-12 / Controla Project
% Made by NDH & LJC

%% Initialize Matlab
clc;
clear all;
% close all;

%% Add Library directory
addpath(genpath('C:\Users\USER\Desktop\Kalman_Filter\Sat_ver2\main'));

%% Initialize setting
params = Params_init(); % Load Parameter struct

X = [ 517; 484; 0; 0; 0; 0 ]; % ( posX, posY, V_X, V_Y, rotZ, rateZ )
Master_point = [ 2400; 2400; 0; 0; 0; 0 ]; % Initialize Mothership position and orientation

X_EKF = X;
X_UKF = X;

P_EKF =  params.EKF.P;
P_UKF =  params.EKF.P;



Rk = params.EKF.Rm;

Q_Fx = 0;
Q_Fy = 0;
Q_Tau = 0;
Qw = diag( [ Q_Fx.^2, Q_Fx.^2, Q_Tau.^2 ]' );

% Simulation setting
Main_Hz = params.System.MainLoopHz;
Sensor_VI_Hz = params.Sensor.VICON_HZ;
PID_control_Hz = Main_Hz;
OPENCV_HZ = params.Sensor.OPENCV_HZ;

sim_dt = 1 / Main_Hz; % [sec]
sim_Time = 160; % [sec]
sim_step = ceil( sim_Time / sim_dt );

% Sensor & Control interval Setting
Sensor_interval = Main_Hz / OPENCV_HZ; % sensor sampling set up
PID_control_interval = Main_Hz / PID_control_Hz;
 
% Time duty initialize
real_time = 0;

% Data log initialize
state_history = zeros(6, sim_step);
target_history = zeros(6, sim_step);
Masterpoint_history = zeros(6, sim_step);

temp3sigma_EKF = zeros(6, sim_step);
temp3sigma_UKF = zeros(6, sim_step);

X_filt_hist_EKF = zeros(6, sim_step);
X_filt_hist_UKF = zeros(6, sim_step);

X_raw_hist = zeros(6, sim_step);
X_predict_hist = zeros(6, sim_step);
debug_control = zeros(9, sim_step);

t = zeros(1, sim_step);

debug_cmd_delta_hist = zeros(9, sim_step);
X_measure = zeros(6, sim_step);

z_true_hist = zeros(3, sim_step);
state_filter_hist = zeros(3, sim_step);
z_measurement_hist = zeros(3, sim_step);


% Main simulation loop
for i = 1 : sim_step
    
    X_target = generate_commands(real_time); % Generate - X_target & tolerance


    if mod(i-1, PID_control_interval) == 0 

        U = Controller_PID_Argument( X, X_target, sim_dt, params );

    end

    [ final_cmd, debug_cmd_delta ]  = Control_Allocator( U, params );

    % Stochastic RK4 simulation
    X = srk4( @vehicle_dynamics_Airbearing_stochastic_debug, X, final_cmd, Qw, sim_dt, params, sim_dt );

    % Generate measurements
    if mod(i-1, Sensor_interval) == 0 
        [ z_AHRS, z_AHRS_true ] = measure_sensor_AHRS( X, Master_point, Rk );
        [ z_Aruco, z_Aruco_true ] = measure_sensor_Aruco( X, Master_point, Rk );
        z = [ z_Aruco; z_AHRS ];
        z_true = [ z_Aruco_true; z_AHRS_true ];

        % EKF update
        [ X_EKF, P_EKF, yhat_EKF, xhat_EKF ] = processDataSIM_Muti_EKF( z, P_EKF, X_EKF, Master_point, final_cmd, params, sim_dt );
        
        % UKF update
        [ X_UKF, P_UKF, yhat_UKF, xhat_UKF ] = processDataSIM_Muti_UKF( z, P_UKF, X_UKF, Master_point, final_cmd, params, sim_dt );
    end

    % data save
    state_history(:, i) = X;
    Masterpoint_history(:, i) = Master_point;
    debug_control(:, i) = final_cmd;
    t(i) = real_time;
    target_history(:, i) = X_target;

    X_filt_hist_EKF(:, i) = X_EKF;
    X_filt_hist_UKF(:, i) = X_UKF;
    temp3sigma_EKF(:, i) = 3 * sqrt(diag(P_EKF));
    temp3sigma_UKF(:, i) = 3 * sqrt(diag(P_UKF));

    real_time = real_time + sim_dt;
end

% Plot results
figure(1);
subplot(3, 1, 1);
plot(t, state_history(1, :), 'b', t, target_history(1, :), 'r--', t, X_filt_hist_EKF(1, :), 'g--', t, X_filt_hist_UKF(1, :), 'm--');
xlabel('[sec]');
ylabel('[mm]');
title('X Position');
legend('True', 'Target', 'EKF', 'UKF');

subplot(3, 1, 2);
plot(t, state_history(2, :), 'b', t, target_history(2, :), 'r--', t, X_filt_hist_EKF(2, :), 'g--', t, X_filt_hist_UKF(2, :), 'm--');
xlabel('[sec]');
ylabel('[mm]');
title('Y Position');
legend('True', 'Target', 'EKF', 'UKF');

subplot(3, 1, 3);
plot(t, rad2deg(state_history(5, :)), 'b', t, rad2deg(target_history(5, :)), 'r--', t, rad2deg(X_filt_hist_EKF(5, :)), 'g--', t, rad2deg(X_filt_hist_UKF(5, :)), 'm--');
xlabel('[sec]');
ylabel('[deg]');
title('Yaw Angle (psi)');
legend('True', 'Target', 'EKF', 'UKF');

% New figure for EKF vs UKF comparison
figure(2);
subplot(3, 1, 1);
plot(t, state_history(1, :) - X_filt_hist_EKF(1, :), 'b-', t, state_history(1, :) - X_filt_hist_UKF(1, :), 'g-');
hold on;
plot(t, temp3sigma_EKF(1, :), 'r-', t, -temp3sigma_EKF(1, :), 'r-');
plot(t, temp3sigma_UKF(1, :), 'm-', t, -temp3sigma_UKF(1, :), 'm-');
xlabel('[sec]');
ylabel('[mm]');
title('X Position Error');
legend('EKF Error', 'UKF Error', 'EKF 3\sigma', 'UKF 3\sigma');

subplot(3, 1, 2);
plot(t, state_history(2, :) - X_filt_hist_EKF(2, :), 'b-', t, state_history(2, :) - X_filt_hist_UKF(2, :), 'g-');
hold on;
plot(t, temp3sigma_EKF(2, :), 'r-', t, -temp3sigma_EKF(2, :), 'r-');
plot(t, temp3sigma_UKF(2, :), 'm-', t, -temp3sigma_UKF(2, :), 'm-');
xlabel('[sec]');
ylabel('[mm]');
title('Y Position Error');
legend('EKF Error', 'UKF Error', 'EKF 3\sigma', 'UKF 3\sigma');

subplot(3, 1, 3);
plot(t, rad2deg(state_history(5, :) - X_filt_hist_EKF(5, :)), 'b-', t, rad2deg(state_history(5, :) - X_filt_hist_UKF(5, :)), 'g-');
hold on;
plot(t, rad2deg(temp3sigma_EKF(5, :)), 'r-', t, -rad2deg(temp3sigma_EKF(5, :)), 'r-');
plot(t, rad2deg(temp3sigma_UKF(5, :)), 'm-', t, -rad2deg(temp3sigma_UKF(5, :)), 'm-');
xlabel('[sec]');
ylabel('[deg]');
title('Yaw Angle Error');
legend('EKF Error', 'UKF Error', 'EKF 3\sigma', 'UKF 3\sigma');

% RMSE calculation
rmse_EKF = sqrt(mean((state_history - X_filt_hist_EKF).^2, 2));
rmse_UKF = sqrt(mean((state_history - X_filt_hist_UKF).^2, 2));

disp('RMSE EKF:');
disp(rmse_EKF);
disp('RMSE UKF:');
disp(rmse_UKF);