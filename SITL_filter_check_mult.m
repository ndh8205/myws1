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
addpath(genpath('C:\Users\DDHD\Desktop\shj\main'));

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
sim_Time = 160; % [sec]
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


% Main simulation loop
for i = 1 : sim_step
    
    X_target = generate_commands(real_time); % Generate - X_target & tolerance


    if mod(i-1, PID_control_interval) == 0 

        U = Controller_PID_Argument( X, X_target, sim_dt, params );

    end

    [ final_cmd, debug_cmd_delta ]  = Control_Allocator( U, params );

    % Stochastic RK4 simulation
    X = srk4( @vehicle_dynamics_Airbearing_stochastic, X, final_cmd, Qw, sim_dt, params, sim_dt );

    % data save
    state_history(:, i) = X;
    Masterpoint_history(:, i) = Master_point;
    debug_control(:, i) = final_cmd;
    t(i) = real_time;
    target_history(:, i) = X_target;

    real_time = real_time + sim_dt;
end

for superLoop = 1:1

X_filt = state_history( :, 1 );
P_ekf =  params.EKF.P;

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

        [ X_filt, P_ekf, y_hat, X_predict ] = processDataSIM_Muti_UKF( z, P_ekf, X_filt, master_point_current, U_ture, params, sim_dt );
        % [ X_filt, P_ekf, y_hat, X_predict ] = processDataSIM_Muti_EKF( z, P_ekf, X_filt, master_point_current, U_ture, params, sim_dt );
        
        X_rel_I = master_point_current(1:2) - X_filt(1:2);
        X_rel_B = [ cos( X_filt(5) ), sin( X_filt(5) ); -sin( X_filt(5) ), cos( X_filt(5) ) ] * X_rel_I;
        rho_k = sqrt( X_rel_B(1)^2 + X_rel_B(2)^2 );
        theta_k = atan2( X_rel_B(2), X_rel_B(1) );
        r_k = X_filt(6);

        % Measurement vector
        h1 = [ rho_k; theta_k; r_k ];

    end

    X_filt_hist( :, k ) = X_filt;
    temp3sigma_EKF( :, k ) = 3 * sqrt( diag( P_ekf ) );
    X_predict_hist( :, k ) = X_predict;
    z_measurement_hist( :, k ) = z;
    z_true_hist( :, k ) = z_true;
    state_filter_hist( : , k ) = h1;

    
    % X_raw_hist( :, k ) = y_hat;

end

%% Plot the results

figure(1);
subplot(3, 1, 1);
plot(t, state_history(1, :), 'b', t, target_history(1, :), 'r--' );
hold on
% plot(t, X_filt_hist(1, :), 'g--' ,t, X_predict_hist(1, :), 'm--')
plot(t, X_filt_hist(1, :), 'g--' )

xlabel('[sec]')
ylabel('[mm]')
title('X Position');
legend('RK4', 'Target', 'filter');

subplot(3, 1, 2);
plot(t, state_history(2, :), 'b', t, target_history(2, :), 'r--' );
hold on
% plot( t, X_filt_hist(2, :), 'g--',t, X_predict_hist(2, :), 'm--' )
plot(t, X_filt_hist(2, :), 'g--' )

xlabel('[sec]')
ylabel('[mm]')
title('Y Position');
legend('RK4', 'Target', 'filter');

subplot(3, 1, 3);
plot(t, rad2deg(state_history(5, :)), 'b', t, rad2deg(target_history(5, :)), 'r--');
hold on
% plot(t, rad2deg(X_filt_hist(5, :)), 'g--', t, rad2deg(X_predict_hist(5, :)), 'm--' )
plot(t, rad2deg(X_filt_hist(5, :)), 'g--')

xlabel('[sec]')
ylabel('[deg]')
title('Yaw Angle (psi)');
legend('RK4', 'Target', 'filter');


figure(2);

caxis = 1;

subplot(3,1,caxis)
plot( t, temp3sigma_EKF( caxis, : ),'r-');
title('Position Error, Estimate')
hold on;grid on ;
plot( t, state_history( caxis, : ) - X_filt_hist( caxis, : ),'b');
hold on;grid on ;
plot( t, -temp3sigma_EKF( caxis, : ),"r-");
grid on ;
hold on;
xlabel('Time(sec)');ylabel('X error [mm]');
ylim([-max(temp3sigma_EKF( caxis, : ))*2 max(temp3sigma_EKF( caxis, : ))*2 ]);

caxis = 2;

subplot(3,1,caxis)
plot( t ,temp3sigma_EKF( caxis, : ),'r-');
title('Position Error, Estimate')
hold on;grid on ;
plot( t , state_history( caxis, : ) - X_filt_hist( caxis, : ),'b');
hold on;grid on ;
plot( t, -temp3sigma_EKF( caxis, : ),"r-");
grid on ;
hold on;
xlabel('Time(sec)');ylabel('Y error [mm]');
ylim([-max(temp3sigma_EKF( caxis, : ))*2 max(temp3sigma_EKF( caxis, : ))*2 ]);

caxis = 5;

subplot(3,1,3)
plot( t, rad2deg( temp3sigma_EKF( caxis, : ) ),'r-');
title('Attitude Error , Estimate')
hold on;grid on ;
plot( t, rad2deg(state_history( caxis, : ) - X_filt_hist( caxis, : )),'b');
hold on;grid on ;
plot( t, -rad2deg( temp3sigma_EKF( caxis, : ) ),"r-");
grid on ;
hold on;
xlabel('Time(sec)');ylabel('psi error [deg]');
ylim([-max(rad2deg( temp3sigma_EKF( caxis, : )))*2 max(rad2deg( temp3sigma_EKF( caxis, : )) )*2 ]);
% 

figure(3);
subplot(3, 1, 1);
plot(t, z_measurement_hist(1, :), 'r:', t, z_true_hist(1, :), 'k', t,  state_filter_hist(1, :), 'g--' );
hold on
xlabel('[sec]')
ylabel('[mm]')
title('rho');
legend('rho mesurement', 'rho ture');

subplot(3, 1, 2);
plot(t, rad2deg(z_measurement_hist(2, :)), 'r:', t, rad2deg(z_true_hist(2, :)), 'k' , t,  rad2deg(state_filter_hist(2, :)), 'g--');
hold on
xlabel('[sec]')
ylabel('[deg]')
title('theta');
legend('theta mesurement', 'theta ture');

subplot(3, 1, 3);
plot(t, rad2deg(z_measurement_hist(3, :)), 'r:', t, rad2deg(z_true_hist(3, :)), 'k' , t,  rad2deg(state_filter_hist(3, :)), 'g--');
hold on
xlabel('[sec]')
ylabel('[deg/s]')
title('r');
legend('r mesurement', 'r ture');
% 
% figure(4);
% set(gcf, 'Position', [100, 100, 800, 600]); % Adjusted figure size for better visibility
% 
% % Create subplot
% h_global = plot(0, 0, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
% hold on;
% h_filtered = plot(0, 0, 'bo', 'MarkerSize', 10, 'LineWidth', 2);
% Target_global = plot(target_history(1, 1), target_history(2, 1), 'ko', 'MarkerSize', 10, 'LineWidth', 2);
% master_point_global = plot(master_point_current(1), master_point_current(2), 'gs', 'MarkerSize', 12, 'LineWidth', 2);
% quiver_raw = quiver(0, 0, 0, 0, 'c', 'LineWidth', 2, 'MaxHeadSize', 0.5);
% quiver_true = quiver(0, 0, 0, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
% quiver_filter = quiver(0, 0, 0, 0, 'm', 'LineWidth', 2, 'MaxHeadSize', 0.5);
% 
% % Add orientation arrows
% true_orientation_arrow = quiver(0, 0, 0, 0, 'r', 'LineWidth', 3, 'MaxHeadSize', 0.6);
% filtered_orientation_arrow = quiver(0, 0, 0, 0, 'b', 'LineWidth', 3, 'MaxHeadSize', 0.6);
% 
% title('Global Frame');
% xlabel('X [mm]');
% ylabel('Y [mm]');
% axis equal;
% grid on;
% legend('True Position', 'Filtered Position', 'Target', 'Mothership', 'Raw Measurement', 'True Value', 'Filtered Value', 'True Orientation', 'Filtered Orientation');
% 
%     % Animation loop
%     for k = 1:sim_step
%         % Global frame
%         X_global = state_history(1:2, k);
%         X_filtered = X_filt_hist(1:2, k);
% 
%         % Vehicle orientation
%         true_orientation = state_history(5, k); % True Yaw angle
%         filtered_orientation = X_filt_hist(5, k); % Filtered Yaw angle
%         true_orientation_vector = [cos(true_orientation); sin(true_orientation)] * 200; % Scale the vector for visibility
%         filtered_orientation_vector = [cos(filtered_orientation); sin(filtered_orientation)] * 200; % Scale the vector for visibility
% 
%         % Raw measurement
%         rho_raw = z_measurement_hist(1, k);
%         theta_raw = z_measurement_hist(2, k) + state_history(5, k); % theta_k + psi
%         obs_vector_raw = [rho_raw * cos(theta_raw); rho_raw * sin(theta_raw)];
% 
%         % True value
%         rho_true = z_true_hist(1, k);
%         theta_true = z_true_hist(2, k) + state_history(5, k); % theta_k + psi
%         obs_vector_true = [rho_true * cos(theta_true); rho_true * sin(theta_true)];
% 
%         % Filtered value
%         rho_filter = state_filter_hist(1, k);
%         theta_filter = state_filter_hist(2, k) + X_filt_hist(5, k); % theta_k + psi
%         obs_vector_filter = [rho_filter * cos(theta_filter); rho_filter * sin(theta_filter)];
% 
%         set(h_global, 'XData', X_global(1), 'YData', X_global(2));
%         set(h_filtered, 'XData', X_filtered(1), 'YData', X_filtered(2));
%         set(Target_global, 'XData', target_history(1, k), 'YData', target_history(2, k));
%         set(master_point_global, 'XData', master_point_current(1), 'YData', master_point_current(2));
%         set(quiver_raw, 'XData', X_global(1), 'YData', X_global(2), ...
%             'UData', obs_vector_raw(1), 'VData', obs_vector_raw(2));
%         set(quiver_true, 'XData', X_global(1), 'YData', X_global(2), ...
%             'UData', obs_vector_true(1), 'VData', obs_vector_true(2));
%         set(quiver_filter, 'XData', X_filtered(1), 'YData', X_filtered(2), ...
%             'UData', obs_vector_filter(1), 'VData', obs_vector_filter(2));
% 
%         % Update orientation arrows
%         set(true_orientation_arrow, 'XData', X_global(1), 'YData', X_global(2), ...
%             'UData', true_orientation_vector(1), 'VData', true_orientation_vector(2));
%         set(filtered_orientation_arrow, 'XData', X_filtered(1), 'YData', X_filtered(2), ...
%             'UData', filtered_orientation_vector(1), 'VData', filtered_orientation_vector(2));
% 
%         xlim([-500 3500]);
%         ylim([-500 3500]);
% 
%         % Update title with current values
%         title(sprintf('Global Frame\nRaw: %.2f mm, %.2f°\nTrue: %.2f mm, %.2f°\nFiltered: %.2f mm, %.2f°\nTrue Orientation: %.2f°\nFiltered Orientation: %.2f°', ...
%             rho_raw, rad2deg(theta_raw), rho_true, rad2deg(theta_true), rho_filter, rad2deg(theta_filter), ...
%             rad2deg(true_orientation), rad2deg(filtered_orientation)));
% 
%         drawnow;
%         % pause(0.01); % Adjust for smoother or faster animation
%     end
end
%% Compute and display RMSE
errors = state_history - X_filt_hist;
RMSE = sqrt(mean(errors.^2, 2));

% Display RMSE for each state variable
state_names = {'X position', 'Y position', 'X velocity', 'Y velocity', 'Psi angle', 'Psi rate'};
for i = 1:length(RMSE)
    if i == 5  % Psi angle in degrees
        RMSE_deg = rad2deg(RMSE(i));
        fprintf('RMSE for %s: %f degrees\n', state_names{i}, RMSE_deg);
    elseif i == 6
        RMSE_degs = rad2deg(RMSE(i));
        fprintf('RMSE for %s: %f degrees\n', state_names{i}, RMSE_degs);
    else
        fprintf('RMSE for %s: %f\n', state_names{i}, RMSE(i));
    end
end