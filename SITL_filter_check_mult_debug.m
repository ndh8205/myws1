%% Initialize Matlab
clc;
clear all;
close all;

%% Add Library directory
addpath(genpath('C:\Users\USER\Desktop\Kalman_Filter\Sat_ver2\main'));

%% Initialize setting
params = Params_init(); % Load Parameter struct

X = [ 517; 484; 0; 0; 0; 0 ]; % ( posX, posY, V_X, V_Y, rotZ, rateZ )
Master_point = [ 2400; 2400; 0; 0; 0; 0 ]; % Initialize Mothership position and orientation

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
sim_Time = 120; % [sec]
sim_step = ceil( sim_Time / sim_dt );

% Sensor & Control interval Setting
Sensor_interval = Main_Hz / OPENCV_HZ; % sensor sampling set up
PID_control_interval = Main_Hz / PID_control_Hz;
 
% Time duty initialize
real_time = 0;

% Data log initialize
state_history = zeros(6, sim_step);
Masterpoint_history = zeros(6, sim_step);
temp3sigma_EKF = zeros(6, sim_step);
X_raw_hist = zeros(6, sim_step);
X_predict_hist = zeros(6, sim_step);
debug_control = zeros(9, sim_step);
t = zeros(1, sim_step);
target_history = zeros(6, sim_step);
debug_cmd_delta_hist = zeros(9, sim_step);
X_measure = zeros(6, sim_step);
X_filt_hist = zeros(6, sim_step);
z_true_hist = zeros(3, sim_step);
state_filter_hist = zeros(3, sim_step);
z_measurement_hist = zeros(3, sim_step);

% Initialize figure(1)
figure(1);

subplot(3,1,1);

h_rk4_x = animatedline('Color', 'b');
h_target_x = animatedline('LineStyle', '--', 'Color', 'r');
h_filter_x = animatedline('LineStyle', '--', 'Color', 'g');
h_predict_x = animatedline('LineStyle', '--', 'Color', 'm');
xlabel('[sec]')
ylabel('[mm]')
title('X Position');
legend('RK4', 'Target', 'Filter', 'Predict');
hold on;

subplot(3,1,2);

h_rk4_y = animatedline('Color', 'b');
h_target_y = animatedline('LineStyle', '--', 'Color', 'r');
h_filter_y = animatedline('LineStyle', '--', 'Color', 'g');
h_predict_y = animatedline('LineStyle', '--', 'Color', 'm');
xlabel('[sec]')
ylabel('[mm]')
title('Y Position');
legend('RK4', 'Target', 'Filter', 'Predict');
hold on;

subplot(3,1,3);

h_rk4_psi = animatedline('Color', 'b');
h_target_psi = animatedline('LineStyle', '--', 'Color', 'r');
h_filter_psi = animatedline('LineStyle', '--', 'Color', 'g');
h_predict_psi = animatedline('LineStyle', '--', 'Color', 'm');
xlabel('[sec]')
ylabel('[deg]')
title('Yaw Angle (psi)');
legend('RK4', 'Target', 'Filter', 'Predict');

% Initialize figure(2)
figure(2);

% Subplot for X error
subplot(3,1,1);
h_error_x = animatedline('Color', 'b');
h_3sigma_x_upper = animatedline('Color', 'r');
h_3sigma_x_lower = animatedline('Color', 'r');
xlabel('Time(sec)');
ylabel('X error [mm]');
title('Position Error, Estimate (X)');
grid on;
hold on;

% Subplot for Y error
subplot(3,1,2);
h_error_y = animatedline('Color', 'b');
h_3sigma_y_upper = animatedline('Color', 'r');
h_3sigma_y_lower = animatedline('Color', 'r');
xlabel('Time(sec)');
ylabel('Y error [mm]');
title('Position Error, Estimate (Y)');
grid on;
hold on;

% Subplot for Psi error
subplot(3,1,3);
h_error_psi = animatedline('Color', 'b');
h_3sigma_psi_upper = animatedline('Color', 'r');
h_3sigma_psi_lower = animatedline('Color', 'r');
xlabel('Time(sec)');
ylabel('Psi error [deg]');
title('Attitude Error, Estimate (Psi)');
grid on;
hold on;
hold on;

% Initialize figure(3)
figure(3);

subplot(3,1,1);
h_z_measurement_rho = plot(NaN, NaN, 'r.', 'MarkerSize', 5);
hold on;
state_filter_hist_rho = animatedline('Color', 'g');
h_z_true_rho = animatedline('Color', 'k');
xlabel('[sec]')
ylabel('[mm]')
title('rho');
legend('rho measurement', 'rho true');
hold on;

subplot(3,1,2);

h_z_measurement_theta = plot(NaN, NaN, 'r.', 'MarkerSize', 5);
hold on;

state_filter_hist_theta = animatedline('Color', 'g');
h_z_true_theta = animatedline('Color', 'k');
xlabel('[sec]')
ylabel('[deg]')
title('theta');
legend('theta measurement', 'theta true');
hold on;

subplot(3,1,3);

h_z_measurement_r = plot(NaN, NaN, 'r.', 'MarkerSize', 5);
hold on;

state_filter_hist_r = animatedline('Color', 'g');
h_z_true_r = animatedline('Color', 'k');
xlabel('[sec]')
ylabel('[deg/s]')
title('r');
legend('r measurement', 'r true');
hold on;

% Main simulation loop
for i = 1 : sim_step

    X_target = generate_commands(real_time); % Generate - X_target & tolerance

    if mod(i-1, PID_control_interval) == 0 
        U = Controller_PID_Argument( X, X_target, sim_dt, params );
    end

    [ final_cmd, debug_cmd_delta ]  = Control_Allocator( U, params );

    % Stochastic RK4 simulation
    X = srk4( @vehicle_dynamics_Airbearing_stochastic, X, final_cmd, Qw, sim_dt, params, sim_dt );

    % Data save
    state_history(:, i) = X;
    Masterpoint_history(:, i) = Master_point;
    debug_control(:, i) = final_cmd;
    t(i) = real_time;
    target_history(:, i) = X_target;

    real_time = real_time + sim_dt;
end

% Initialize EKF variables
X_filt = state_history( :, 1 );
P_ekf =  params.EKF.P;

% Second loop for EKF processing and plotting
for k = 1 : sim_step

    if mod(k-1, Sensor_interval) == 0 

        X_ture = state_history( :, k );
        U_ture = debug_control( :, k );
        X_target_ekf = target_history( :, k );
        master_point_current = Masterpoint_history( :, k );

        [ z_AHRS, z_AHRS_true ] = measure_sensor_AHRS( X_ture, master_point_current, Rk );
        [ z_Aruco, z_Aruco_true ] = measure_sensor_Aruco( X_ture, master_point_current, Rk );

        z = [ z_Aruco; z_AHRS ];
        z_true = [ z_Aruco_true; z_AHRS_true ];

        [ X_filt, P_ekf, ye_hat, X_predict_ekf ] = processDataSIM_Muti_EKF( z, P_ekf, X_filt, master_point_current, U_ture, params, sim_dt );
        % [ X_filt, P_ekf, yu_hat, X_predict_ekf ] = processDataSIM_Muti_UKF( z, P_ekf, X_filt, master_point_current, U_ture, params, sim_dt );
        
        X_rel_I = Master_point(1:2) - X_filt(1:2);
        X_rel_B = [ cos( X_filt(5) ), sin( X_filt(5) ); -sin( X_filt(5) ), cos( X_filt(5) ) ] * X_rel_I;
        rho_k = sqrt( X_rel_B(1)^2 + X_rel_B(2)^2 );
        theta_k = atan2( X_rel_B(2), X_rel_B(1) );
        r_k = X_filt(6);

        % Measurement vector
        h1 = [ rho_k; theta_k; r_k ];

        % Save measurement data
        z_measurement_hist( :, k ) = z;
        z_true_hist( :, k ) = z_true;
        state_filter_hist( : , k ) = h1;
    end

    X_filt_hist( :, k ) = X_filt;
    temp3sigma_EKF( :, k ) = 3 * sqrt( diag( P_ekf ) );
    X_predict_hist( :, k ) = X_predict_ekf;

    % Update figure(1)
    subplot(3,1,1);
    addpoints(h_rk4_x, t(k), state_history(1, k));
    addpoints(h_target_x, t(k), target_history(1, k));
    addpoints(h_filter_x, t(k), X_filt_hist(1, k));
    addpoints(h_predict_x, t(k), X_predict_hist(1, k));

    subplot(3,1,2);
    addpoints(h_rk4_y, t(k), state_history(2, k));
    addpoints(h_target_y, t(k), target_history(2, k));
    addpoints(h_filter_y, t(k), X_filt_hist(2, k));
    addpoints(h_predict_y, t(k), X_predict_hist(2, k));

    subplot(3,1,3);
    addpoints(h_rk4_psi, t(k), rad2deg(state_history(5, k)));
    addpoints(h_target_psi, t(k), rad2deg(target_history(5, k)));
    addpoints(h_filter_psi, t(k), rad2deg(X_filt_hist(5, k)));
    addpoints(h_predict_psi, t(k), rad2deg(X_predict_hist(5, k)));


    % Update figure(2)
    % For X error
    subplot(3,1,1);
    error_x = state_history(1, k) - X_filt_hist(1, k);
    addpoints(h_error_x, t(k), error_x);
    addpoints(h_3sigma_x_upper, t(k), temp3sigma_EKF(1, k));
    addpoints(h_3sigma_x_lower, t(k), -temp3sigma_EKF(1, k));
    
    % For Y error
    subplot(3,1,2);
    error_y = state_history(2, k) - X_filt_hist(2, k);
    addpoints(h_error_y, t(k), error_y);
    addpoints(h_3sigma_y_upper, t(k), temp3sigma_EKF(2, k));
    addpoints(h_3sigma_y_lower, t(k), -temp3sigma_EKF(2, k));
    
    % For Psi error
    subplot(3,1,3);
    error_psi = rad2deg(state_history(5, k) - X_filt_hist(5, k));
    addpoints(h_error_psi, t(k), error_psi);
    addpoints(h_3sigma_psi_upper, t(k), rad2deg(temp3sigma_EKF(5, k)));
    addpoints(h_3sigma_psi_lower, t(k), -rad2deg(temp3sigma_EKF(5, k)));



    % Update figure(3)
    if mod(k-1, Sensor_interval) == 0 
        time = t(k);

        % 측정값 (노이즈가 있는 값) 업데이트 - 점으로 표시
        set(h_z_measurement_rho, 'XData', [get(h_z_measurement_rho, 'XData') time], 'YData', [get(h_z_measurement_rho, 'YData') z_measurement_hist(1, k)]);
        set(h_z_measurement_theta, 'XData', [get(h_z_measurement_theta, 'XData') time], 'YData', [get(h_z_measurement_theta, 'YData') rad2deg(z_measurement_hist(2, k))]);
        set(h_z_measurement_r, 'XData', [get(h_z_measurement_r, 'XData') time], 'YData', [get(h_z_measurement_r, 'YData') rad2deg(z_measurement_hist(3, k))]);

        % rho
        subplot(3,1,1);
        addpoints(state_filter_hist_rho, time, state_filter_hist(1, k));
        addpoints(h_z_true_rho, time, z_true_hist(1, k));

        % theta
        subplot(3,1,2);
        addpoints(state_filter_hist_theta, time, rad2deg(state_filter_hist(2, k)));
        addpoints(h_z_true_theta, time, rad2deg(z_true_hist(2, k)));

        % r
        subplot(3,1,3);
        addpoints(state_filter_hist_r, time, rad2deg(state_filter_hist(3, k)));
        addpoints(h_z_true_r, time, rad2deg(z_true_hist(3, k)));
    end

    drawnow limitrate;

    disp(k*sim_dt/1)

end

