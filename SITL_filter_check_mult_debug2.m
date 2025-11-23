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
temp3sigma_UKF = zeros(6, sim_step);
X_raw_hist = zeros(6, sim_step);
X_predict_hist_ekf = zeros(6, sim_step);
X_predict_hist_ukf = zeros(6, sim_step);
debug_control = zeros(9, sim_step);
t = zeros(1, sim_step);
target_history = zeros(6, sim_step);
debug_cmd_delta_hist = zeros(9, sim_step);
X_measure = zeros(6, sim_step);
X_filt_hist_ekf = zeros(6, sim_step);
X_filt_hist_ukf = zeros(6, sim_step);
z_true_hist = zeros(3, sim_step);
state_filter_hist_ekf = zeros(3, sim_step);
state_filter_hist_ukf = zeros(3, sim_step);
z_measurement_hist = zeros(3, sim_step);

% Initialize figures
figure(1);

subplot(3,1,1);
h_rk4_x = animatedline('Color', 'k');
h_target_x = animatedline('LineStyle', '-', 'Color', 'r');
h_filter_x_ekf = animatedline('LineStyle', '-', 'Color', 'b');
h_filter_x_ukf = animatedline('LineStyle', '-', 'Color', 'g');
h_predict_x_ekf = animatedline('LineStyle', '--', 'Color', 'm');
h_predict_x_ukf = animatedline('LineStyle', '--', 'Color', 'c');
xlabel('[sec]')
ylabel('[mm]')
title('X Position');
legend('RK4', 'Target', 'EKF', 'UKF', 'EKF Predict', 'UKF Predict');
hold on;

subplot(3,1,2);
h_rk4_y = animatedline('Color', 'k');
h_target_y = animatedline('LineStyle', '-', 'Color', 'r');
h_filter_y_ekf = animatedline('LineStyle', '-', 'Color', 'b');
h_filter_y_ukf = animatedline('LineStyle', '-', 'Color', 'g');
h_predict_y_ekf = animatedline('LineStyle', '--', 'Color', 'm');
h_predict_y_ukf = animatedline('LineStyle', '--', 'Color', 'c');
xlabel('[sec]')
ylabel('[mm]')
title('Y Position');
legend('RK4', 'Target', 'EKF', 'UKF', 'EKF Predict', 'UKF Predict');
hold on;

subplot(3,1,3);
h_rk4_psi = animatedline('Color', 'k');
h_target_psi = animatedline('LineStyle', '-', 'Color', 'r');
h_filter_psi_ekf = animatedline('LineStyle', '-', 'Color', 'b');
h_filter_psi_ukf = animatedline('LineStyle', '-', 'Color', 'g');
h_predict_psi_ekf = animatedline('LineStyle', '--', 'Color', 'm');
h_predict_psi_ukf = animatedline('LineStyle', '--', 'Color', 'c');
xlabel('[sec]')
ylabel('[deg]')
title('Yaw Angle (psi)');
legend('RK4', 'Target', 'EKF', 'UKF', 'EKF Predict', 'UKF Predict');
hold on;

figure(3);

subplot(3,1,1);
h_z_measurement_rho = plot(NaN, NaN, 'r.', 'MarkerSize', 5);
hold on;
state_filter_hist_rho_ekf = animatedline('Color', 'g');
state_filter_hist_rho_ukf = animatedline('Color', 'b');
h_z_true_rho = animatedline('Color', 'k');
xlabel('[sec]')
ylabel('[mm]')
title('rho');
legend('rho measurement', 'EKF', 'UKF', 'rho true');
hold on;

subplot(3,1,2);
h_z_measurement_theta = plot(NaN, NaN, 'r.', 'MarkerSize', 5);
hold on;
state_filter_hist_theta_ekf = animatedline('Color', 'g');
state_filter_hist_theta_ukf = animatedline('Color', 'b');
h_z_true_theta = animatedline('Color', 'k');
xlabel('[sec]')
ylabel('[deg]')
title('theta');
legend('theta measurement', 'EKF', 'UKF', 'theta true');
hold on;

subplot(3,1,3);
h_z_measurement_r = plot(NaN, NaN, 'r.', 'MarkerSize', 5);
hold on;
state_filter_hist_r_ekf = animatedline('Color', 'g');
state_filter_hist_r_ukf = animatedline('Color', 'b');
h_z_true_r = animatedline('Color', 'k');
xlabel('[sec]')
ylabel('[deg/s]')
title('r');
legend('r measurement', 'EKF', 'UKF', 'r true');
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

% Initialize EKF and UKF variables
X_filt_ekf = state_history(:, 1);
X_filt_ukf = state_history(:, 1);
P_ekf = params.EKF.P;
P_ukf = params.UKF.P;

% Second loop for EKF and UKF processing and plotting
for k = 1 : sim_step
    if mod(k-1, Sensor_interval) == 0 
        X_true = state_history(:, k);
        U_true = debug_control(:, k);
        X_target_ekf = target_history(:, k);
        master_point_current = Masterpoint_history(:, k);

        [ z_AHRS, z_AHRS_true ] = measure_sensor_AHRS( X_true, master_point_current, Rk );
        [ z_Aruco, z_Aruco_true ] = measure_sensor_Aruco( X_true, master_point_current, Rk );

        z = [ z_Aruco; z_AHRS ];
        z_true = [ z_Aruco_true; z_AHRS_true ];

        [ X_filt_ekf, P_ekf, ye_hat, X_predict_ekf ] = processDataSIM_Muti_EKF( z, P_ekf, X_filt_ekf, master_point_current, U_true, params, sim_dt );
        [ X_filt_ukf, P_ukf, yu_hat, X_predict_ukf ] = processDataSIM_Muti_UKF( z, P_ukf, X_filt_ukf, master_point_current, U_true, params, sim_dt );
        
        % Calculate measurement vector for EKF
        X_rel_I_ekf = Master_point(1:2) - X_filt_ekf(1:2);
        X_rel_B_ekf = [ cos( X_filt_ekf(5) ), -sin( X_filt_ekf(5) ); sin( X_filt_ekf(5) ), cos( X_filt_ekf(5) ) ] * X_rel_I_ekf;
        rho_k_ekf = sqrt( X_rel_B_ekf(1)^2 + X_rel_B_ekf(2)^2 );
        theta_k_ekf = atan2( X_rel_I_ekf(2), X_rel_I_ekf(1) ) - X_filt_ekf(5);
        r_k_ekf = X_filt_ekf(6);
        h1_ekf = [ rho_k_ekf; theta_k_ekf; r_k_ekf ];

        % Calculate measurement vector for UKF
        X_rel_I_ukf = Master_point(1:2) - X_filt_ukf(1:2);
        X_rel_B_ukf = [ cos( X_filt_ukf(5) ), -sin( X_filt_ukf(5) ); sin( X_filt_ukf(5) ), cos( X_filt_ukf(5) ) ] * X_rel_I_ukf;
        rho_k_ukf = sqrt( X_rel_B_ukf(1)^2 + X_rel_B_ukf(2)^2 );
        theta_k_ukf = atan2( X_rel_I_ukf(2), X_rel_I_ukf(1) ) - X_filt_ukf(5);
        r_k_ukf = X_filt_ukf(6);
        h1_ukf = [ rho_k_ukf; theta_k_ukf; r_k_ukf ];

        % Save measurement data
        z_measurement_hist(:, k) = z;
        z_true_hist(:, k) = z_true;
        state_filter_hist_ekf(:, k) = h1_ekf;
        state_filter_hist_ukf(:, k) = h1_ukf;
    end

    X_filt_hist_ekf(:, k) = X_filt_ekf;
    X_filt_hist_ukf(:, k) = X_filt_ukf;
    temp3sigma_EKF(:, k) = 3 * sqrt(diag(P_ekf));
    temp3sigma_UKF(:, k) = 3 * sqrt(diag(P_ukf));
    X_predict_hist_ekf(:, k) = X_predict_ekf;
    X_predict_hist_ukf(:, k) = X_predict_ukf;


    x_max = t(k);
    x_min = x_max - 5;
    if x_min < 0
        x_min = 0;
    end

    if x_max <= x_min
        x_max = x_min + 1e-6; 
    end

    % Update figure(1)
    subplot(3,1,1);
    addpoints(h_rk4_x, t(k), state_history(1, k));
    addpoints(h_target_x, t(k), target_history(1, k));
    addpoints(h_filter_x_ekf, t(k), X_filt_hist_ekf(1, k));
    addpoints(h_filter_x_ukf, t(k), X_filt_hist_ukf(1, k));
    addpoints(h_predict_x_ekf, t(k), X_predict_hist_ekf(1, k));
    addpoints(h_predict_x_ukf, t(k), X_predict_hist_ukf(1, k));
    axis tight;
    xlim([x_min x_max]);


    subplot(3,1,2);
    addpoints(h_rk4_y, t(k), state_history(2, k));
    addpoints(h_target_y, t(k), target_history(2, k));
    addpoints(h_filter_y_ekf, t(k), X_filt_hist_ekf(2, k));
    addpoints(h_filter_y_ukf, t(k), X_filt_hist_ukf(2, k));
    addpoints(h_predict_y_ekf, t(k), X_predict_hist_ekf(2, k));
    addpoints(h_predict_y_ukf, t(k), X_predict_hist_ukf(2, k));
    axis tight;
    xlim([x_min x_max]);
    
    subplot(3,1,3);
    addpoints(h_rk4_psi, t(k), rad2deg(state_history(5, k)));
    addpoints(h_target_psi, t(k), rad2deg(target_history(5, k)));
    addpoints(h_filter_psi_ekf, t(k), rad2deg(X_filt_hist_ekf(5, k)));
    addpoints(h_filter_psi_ukf, t(k), rad2deg(X_filt_hist_ukf(5, k)));
    axis tight;
    xlim([x_min x_max]);
   

    % Update figure(3)
    if mod(k-1, Sensor_interval) == 0 
        time = t(k);

        % 측정값 (노이즈가 있는 값) 업데이트 - 점으로 표시
        set(h_z_measurement_rho, 'XData', [get(h_z_measurement_rho, 'XData') time], 'YData', [get(h_z_measurement_rho, 'YData') z_measurement_hist(1, k)]);
        set(h_z_measurement_theta, 'XData', [get(h_z_measurement_theta, 'XData') time], 'YData', [get(h_z_measurement_theta, 'YData') rad2deg(z_measurement_hist(2, k))]);
        set(h_z_measurement_r, 'XData', [get(h_z_measurement_r, 'XData') time], 'YData', [get(h_z_measurement_r, 'YData') rad2deg(z_measurement_hist(3, k))]);

        % rho
        subplot(3,1,1);
        addpoints(state_filter_hist_rho_ekf, time, state_filter_hist_ekf(1, k));
        addpoints(state_filter_hist_rho_ukf, time, state_filter_hist_ukf(1, k));
        addpoints(h_z_true_rho, time, z_true_hist(1, k));
        axis tight;
        xlim([x_min x_max]); % X축 범위 설정


        % theta
        subplot(3,1,2);
        addpoints(state_filter_hist_theta_ekf, time, rad2deg(state_filter_hist_ekf(2, k)));
        addpoints(state_filter_hist_theta_ukf, time, rad2deg(state_filter_hist_ukf(2, k)));
        addpoints(h_z_true_theta, time, rad2deg(z_true_hist(2, k)));
        axis tight;
        xlim([x_min x_max]); % X축 범위 설정


        % r
        subplot(3,1,3);
        addpoints(state_filter_hist_r_ekf, time, rad2deg(state_filter_hist_ekf(3, k)));
        addpoints(state_filter_hist_r_ukf, time, rad2deg(state_filter_hist_ukf(3, k)));
        addpoints(h_z_true_r, time, rad2deg(z_true_hist(3, k)));
        axis tight;
        xlim([x_min x_max]); % X축 범위 설정

    end

    drawnow limitrate;

    % disp(['Progress: ' num2str(k*sim_dt/sim_Time*100, '%.2f') '%'])
end

% Plot error comparison
figure(4);
subplot(3,1,1);
plot(t, abs(state_history(1,:) - X_filt_hist_ekf(1,:)), 'r', 'DisplayName', 'EKF Error');
hold on;
plot(t, abs(state_history(1,:) - X_filt_hist_ukf(1,:)), 'b', 'DisplayName', 'UKF Error');
xlabel('Time [s]');
ylabel('X Position Error [mm]');
legend('show');
title('X Position Error Comparison');

subplot(3,1,2);
plot(t, abs(state_history(2,:) - X_filt_hist_ekf(2,:)), 'r', 'DisplayName', 'EKF Error');
hold on;
plot(t, abs(state_history(2,:) - X_filt_hist_ukf(2,:)), 'b', 'DisplayName', 'UKF Error');
xlabel('Time [s]');
ylabel('Y Position Error [mm]');
legend('show');
title('Y Position Error Comparison');

subplot(3,1,3);
plot(t, abs(rad2deg(state_history(5,:) - X_filt_hist_ekf(5,:))), 'r', 'DisplayName', 'EKF Error');
hold on;
plot(t, abs(rad2deg(state_history(5,:) - X_filt_hist_ukf(5,:))), 'b', 'DisplayName', 'UKF Error');
xlabel('Time [s]');
ylabel('Yaw Angle Error [deg]');
legend('show');
title('Yaw Angle Error Comparison');

% Calculate and display RMSE
rmse_ekf = sqrt(mean((state_history - X_filt_hist_ekf).^2, 2));
rmse_ukf = sqrt(mean((state_history - X_filt_hist_ukf).^2, 2));

disp('Root Mean Square Error (RMSE):');
disp(['EKF - X: ' num2str(rmse_ekf(1)) ' mm, Y: ' num2str(rmse_ekf(2)) ' mm, Yaw: ' num2str(rad2deg(rmse_ekf(5))) ' deg']);
disp(['UKF - X: ' num2str(rmse_ukf(1)) ' mm, Y: ' num2str(rmse_ukf(2)) ' mm, Yaw: ' num2str(rad2deg(rmse_ukf(5))) ' deg']);

% Save results
save('simulation_results.mat', 'state_history', 'X_filt_hist_ekf', 'X_filt_hist_ukf', 't', 'rmse_ekf', 'rmse_ukf');