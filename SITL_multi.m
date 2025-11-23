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

for superloop = 1:1
%% Initialize setting
params = Params_init(); % Load Parameter struct

X = [ 517; 484; 0; 0; 0; 0 ]; % ( posX, posY, V_X, V_Y, rotZ, rateZ )
X_true = X;
Xk = X;
z = X; % ( posX, posY, V_X, V_Y, rotZ, rateZ )
final_cmd = [ zeros( 8,1 ); 49 ]; % ( T1 T2 T3 T4 T5 T6 T7 T8 RW )

Rk = params.EKF.R;
Rm = params.EKF.Rm;

Q_Fx = 0;
Q_Fy = 0;
Q_Tau = 0;
Qw = diag( [ Q_Fx.^2, Q_Fx.^2, Q_Tau.^2 ]' );

P = diag( [ 10.01, 10.01, 10, 10, deg2rad(100), deg2rad(100), ] );



% Simulation setting
Main_Hz = params.System.MainLoopHz;
Sensor_VI_Hz = params.Sensor.VICON_HZ;
PID_control_Hz = Main_Hz;
AHRS_HZ = params.Sensor.AHRS_HZ;

sim_dt = 1 / Main_Hz; % [sec]
sim_Time = 60; % [sec]
sim_step = ceil( sim_Time / sim_dt );

% Sensor & Control interval Setting
Sensor_VI_interval = Main_Hz / AHRS_HZ; % sensor sampling set up
PID_control_interval = Main_Hz / PID_control_Hz; 
 
% Time duty initialize
real_time = 0;

% Data log initialize (메모리 사전 할당)
state_history = zeros(6, sim_step);
state_filter_hist = zeros(3, sim_step);
temp3sigma_EKF = zeros(6, sim_step);
inovation_history = zeros(6, sim_step);
X_raw_hist = zeros(3, sim_step);
X_predict_hist = zeros(6, sim_step);
debug_control = zeros(8, sim_step);
t = zeros(1, sim_step);
target_history = zeros(6, sim_step);
debug_cmd_delta_hist = zeros(8, sim_step);
z_true_hist = zeros(3, sim_step);

% Main simulation loop
for i = 1 : sim_step

    tic;
    
    X_target = generate_commands( real_time );

    if mod(i-1, Sensor_VI_interval) == 0 

        % z_VICON = measure_sensor_VICON( Xk, X_target, Rk );
        [ z_AHRS, z_AHRS_true ] = measure_sensor_AHRS( Xk, X_target, Rm );
        [ z_Aruco, z_Aruco_true ] = measure_sensor_Aruco( Xk, X_target, Rm );
        
        z = [ z_Aruco; z_AHRS ];
        z_true = [ z_Aruco_true; z_AHRS_true ];

        [ Xk, P, yhat, X_predict ] = processDataSIM_Muti_EKF( z, P, Xk, X_target, final_cmd, params, sim_dt );

        X_rel_I = X_target(1:2) - Xk(1:2);
        X_rel_B = [ cos( Xk(5) ), -sin( Xk(5) ); sin( Xk(5) ), cos( Xk(5) ) ] * X_rel_I;
        rho_k = sqrt( X_rel_B(1)^2 + X_rel_B(2)^2 );
        theta_k = atan2( X_rel_I(2), X_rel_I(1) ) - Xk(5);
        r_k = Xk(6);

        % Measurement vector
        h1 = [ rho_k; theta_k; r_k ];

    end

    if mod(i-1, PID_control_interval) == 0 

        U = Controller_PID_Argument( Xk, X_target, sim_dt, params );

    end

    [ final_cmd, debug_cmd_delta ]  = Control_Allocator( U, params );

    % state_filter_hist( : , i ) = Xk;
    state_filter_hist( : , i ) = h1;

    
    % Stochastic RK4 simulation
    Xk = srk4( @vehicle_dynamics_Airbearing_stochastic, Xk, zeros(9,1), Qw, sim_dt, params, sim_dt );

    % data save
    state_history( :, i ) = Xk;
    debug_control( :, i ) = final_cmd( 1 : 8 );
    X_predict_hist(:, i) = X_predict;
    t( i ) = real_time;
    target_history( :, i ) = X_target;
    temp3sigma_EKF( :, i ) = 3 * sqrt( diag( P ) );
    X_raw_hist( :, i ) = z;
    % inovation_history( : , i ) = yhat;
    z_true_hist( :, i ) = z_true;
    
    real_time = real_time + sim_dt;

    sim_timer = toc;

end

%% Plot the results

figure(1);
subplot(3, 1, 1);
plot(t, state_history(1, :), 'b', t, target_history(1, :), 'r--' );
hold on
plot(t, X_predict_hist(1, :), 'g--' )
xlabel('[sec]')
ylabel('[mm]')
title('X Position');
legend('RK4', 'Target', 'predict');

subplot(3, 1, 2);
plot(t, state_history(2, :), 'b', t, target_history(2, :), 'r--' );
hold on
plot(t, X_predict_hist(2, :), 'g--' )
xlabel('[sec]')
ylabel('[mm]')
title('Y Position');
legend('RK4', 'Target', 'predict');

subplot(3, 1, 3);
plot(t, rad2deg(state_history(5, :)), 'b', t, rad2deg(target_history(5, :)), 'r--' );
hold on
plot(t, rad2deg(X_predict_hist(5, :)), 'g--' )
xlabel('[sec]')
ylabel('[deg]')
title('Yaw Angle (psi)');
legend('RK4', 'Target', 'predict');
% 
% figure(2);
% imagesc(t, 1:8, debug_control);
% colormap([92, 136, 196; 255, 253, 181] / 255);
% xlabel('Time [sec]');
% ylabel('Relay Number');
% title('Relay States Heatmap');
% yticks(1:8);
% grid on;
% c = colorbar;
% c.Ticks = [0.25 0.75];
% c.TickLabels = {'OFF', 'ON'};

figure(3);

caxis = 1;

subplot(2,1,caxis)
plot( t, temp3sigma_EKF( caxis, : ),'m-');
title('Position Error, Estimate')
hold on;grid on ;
plot( t, state_filter_hist( caxis, : ) - z_true_hist( caxis, : ),'g');
hold on;grid on ;
plot( t, -temp3sigma_EKF( caxis, : ),"m-");
grid on ;
hold on;
xlabel('Time(sec)');ylabel('X error [mm]');
ylim([-max(temp3sigma_EKF( caxis, : ))*2 max(temp3sigma_EKF( caxis, : ))*2 ]);

caxis = 2;

subplot(2,1,caxis)
plot( t ,temp3sigma_EKF( caxis, : ),'m-');
title('Position Error, Estimate')
hold on;grid on ;
plot( t , state_filter_hist( caxis, : ) - z_true_hist( caxis, : ),'g');
hold on;grid on ;
plot( t, -temp3sigma_EKF( caxis, : ),"m-");
grid on ;
hold on;
xlabel('Time(sec)');ylabel('Y error [mm]');
ylim([-max(temp3sigma_EKF( caxis, : ))*2 max(temp3sigma_EKF( caxis, : ))*2 ]);

caxis = 3;

subplot(2,1,caxis)
plot( t, rad2deg( temp3sigma_EKF( caxis, : ) ),'m-');
title('Attitude Error , Estimate')
hold on;grid on ;
plot( t, rad2deg( state_filter_hist( caxis, : ) - z_true_hist( caxis, : ) ),'g');
hold on;grid on ;
plot( t, -rad2deg( temp3sigma_EKF( caxis, : ) ),"m-");
grid on ;
hold on;
xlabel('Time(sec)');ylabel('psi error [deg]');
ylim([-max(rad2deg( temp3sigma_EKF( caxis, : )))*2 max(rad2deg( temp3sigma_EKF( caxis, : )) )*2 ]);


end