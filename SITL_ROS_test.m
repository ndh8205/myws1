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
addpath(genpath('C:\Users\DDHD\Desktop\sat_hw_ver_Nonlinear\main'));

% for superloop = 1:1
%% Initialize setting
params = Params_init(); % Load Parameter struct

X = [ 517; 484; 0; 0; 0; 0 ]; % ( posX, posY, V_X, V_Y, rotZ, rateZ )
master_point = [2400; 2400; 0; 0; 0; 0]; % Initialize Mothership position and orientation
X_true = X;
Xk = X;
z = X; % ( posX, posY, V_X, V_Y, rotZ, rateZ )
final_cmd = [ zeros( 8,1 ); 49 ]; % ( T1 T2 T3 T4 T5 T6 T7 T8 RW )

P = params.EKF.P;
Rk = params.EKF.R;
Rm = params.EKF.Rm;

Q_Fx = 1;
Q_Fy = 1;
Q_Tau = 1;
Qw = diag( [ Q_Fx.^2, Q_Fx.^2, Q_Tau.^2 ]' );


% Simulation setting
Main_Hz = params.System.MainLoopHz;
Sensor_VI_Hz = params.Sensor.VICON_HZ;
PID_control_Hz = Main_Hz;
AHRS_HZ = params.Sensor.AHRS_HZ;
OPENCV_HZ = params.Sensor.OPENCV_HZ;

sim_dt = 1 / Main_Hz; % [sec]
sim_Time = 8; % [sec]
sim_step = ceil( sim_Time / sim_dt );

% Sensor & Control interval Setting
Sensor_VI_interval = Main_Hz / OPENCV_HZ; % sensor sampling set up
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
inovation_history = zeros(3, sim_step);


% Main simulation loop
for i = 1 : sim_step

    tic;
    
    X_target = generate_commands( real_time );

    % % if mod(i-1, Sensor_VI_interval) == 0 
    % % 
    % %     % z= measure_sensor( Xk, X_target, Rk );
    % %     [ z_AHRS, z_AHRS_true ] = measure_sensor_AHRS( Xk, master_point, Rm );
    % %     [ z_Aruco, z_Aruco_true ] = measure_sensor_Aruco( Xk, master_point, Rm );
    % % 
    % %     z = [ z_Aruco; z_AHRS ];
    % %     z_true = [ z_Aruco_true; z_AHRS_true ];
    % % 
    % %     [Xk, P, yhat, X_predict] = processDataSIM_Muti_EKF(z, P, Xk, master_point, final_cmd, params, sim_dt);
    % % 
    % %    % state filter history
    % %     X_rel_I = master_point(1:2) - Xk(1:2);
    % %     X_rel_B = [cos(Xk(5)), sin(Xk(5)); -sin(Xk(5)), cos(Xk(5))] * X_rel_I;
    % %     rho_k = sqrt(X_rel_B(1)^2 + X_rel_B(2)^2);
    % %     theta_k = atan2(X_rel_B(2), X_rel_B(1));
    % %     r_k = Xk(6);
    % % 
    % %     % Measurement vector
    % %     h1 = [ rho_k; theta_k; r_k ];

    % end

    if mod(i-1, PID_control_interval) == 0 

        U = Controller_PID_Argument( Xk, X_target, sim_dt, params );

    end

    [ final_cmd, debug_cmd_delta ]  = Control_Allocator( U, params );
    
    % Stochastic RK4 simulation zeros(8,1) final_cmd
    Xk = srk4( @vehicle_dynamics_Airbearing_stochastic, Xk, final_cmd, zeros(3,1), sim_dt, params, sim_dt );

    % data save
    state_history( :, i ) = Xk;
    % state_filter_hist( : , i ) = h1;
    % X_predict_hist(:, i) = X_predict;
    t( i ) = real_time;
    target_history( :, i ) = X_target;
    % temp3sigma_EKF( :, i ) = 3 * sqrt( diag( P ) );
    % z_measurement_hist(:, i) = z;

    % inovation_history( : , i ) = yhat;
    % z_true_hist( :, i ) = z_true;

    
    
    real_time = real_time + sim_dt;

    sim_timer = toc;

end

%% Plot the results

figure(1);
subplot(3, 1, 1);
plot(t, state_history(1, :), 'b', t, target_history(1, :), 'r--');
hold on
xlabel('[sec]')
ylabel('[mm]')
title('X Position');
legend('EKF-Srk1 Estimate', 'Target' );

subplot(3, 1, 2);
plot(t, state_history(2, :), 'b', t, target_history(2, :), 'r--');
hold on
xlabel('[sec]')
ylabel('[mm]')
title('Y Position');
legend('EKF-Srk1 Estimate', 'Target' );

subplot(3, 1, 3);
plot(t, rad2deg(state_history(5, :)), 'b', t, rad2deg(target_history(5, :)), 'r--');
hold on
xlabel('[sec]')
ylabel('[deg]')
title('Yaw Angle (psi)');
legend('EKF-Srk1 Estimate', 'Target' );
