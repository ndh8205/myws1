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
X_true = X;
Xk = X;
z = X; % ( posX, posY, V_X, V_Y, rotZ, rateZ )
final_cmd = [ zeros( 8,1 ); 49 ]; % ( T1 T2 T3 T4 T5 T6 T7 T8 RW )

Rk = params.EKF.R;

Q_Fx = 0.1;
Q_Fy = 0.1;
Q_Tau = 0.1;
Qw = diag( [ Q_Fx.^2, Q_Fx.^2, Q_Tau.^2 ]' );

P = params.EKF.P;

% Simulation setting
Main_Hz = params.System.MainLoopHz;
Sensor_VI_Hz = params.Sensor.VICON_HZ;
PID_control_Hz = Main_Hz;

sim_dt = 1 / Main_Hz; % [sec]
sim_Time = 160; % [sec]
sim_step = ceil( sim_Time / sim_dt );

% Sensor & Control interval Setting
Sensor_VI_interval = Main_Hz / Sensor_VI_Hz; % sensor sampling set up
PID_control_interval = Main_Hz / PID_control_Hz; 
 
% Time duty initialize
real_time = 0;

% Data log initialize (메모리 사전 할당)
state_history = zeros(6, sim_step);
state_filter_hist = zeros(6, sim_step);
temp3sigma_EKF = zeros(6, sim_step);
inovation_history = zeros(6, sim_step);
X_raw_hist = zeros(6, sim_step);
X_predict_hist = zeros(6, sim_step);
debug_control = zeros(8, sim_step);
t = zeros(1, sim_step);
target_history = zeros(6, sim_step);
debug_cmd_delta_hist = zeros(8, sim_step);

% Main simulation loop
for i = 1 : sim_step

    tic;
    
    X_target = generate_commands( real_time );

    if mod(i-1, Sensor_VI_interval) == 0 

        z = Xk + randn( 6, 1 ) .* [ Rk(1,1), Rk(2,2), Rk(3,3), Rk(4,4), Rk(5,5), Rk(6,6) ]';

        [ Xk, P, yhat, X_predict ] = processDataSIM_VI( z, P, Xk, X_target, final_cmd, params, 1/Sensor_VI_Hz );

    end

    if mod(i-1, PID_control_interval) == 0 

        U = Controller_PID_Argument( Xk, X_target, sim_dt, params );

    end

    [ final_cmd, debug_cmd_delta ]  = Control_Allocator( U, params );

    state_filter_hist( : , i ) = Xk;
    
    % Stochastic RK4 simulation
    Xk = srk4( @vehicle_dynamics_Airbearing_stochastic, Xk, final_cmd, Qw, sim_dt, params, sim_dt );

    % data save
    state_history( :, i ) = Xk;
    debug_control( :, i ) = final_cmd( 1 : 8 );
    X_predict_hist(:, i) = X_predict;
    t( i ) = real_time;
    target_history( :, i ) = X_target;
    temp3sigma_EKF( :, i ) = 3 * sqrt( diag( P ) );
    X_raw_hist( :, i ) = z;
    inovation_history( : , i ) = yhat;
    
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
figure(2);
imagesc(t, 1:8, debug_control);
colormap([92, 136, 196; 255, 253, 181] / 255);
xlabel('Time [sec]');
ylabel('Relay Number');
title('Relay States Heatmap');
yticks(1:8);
grid on;
c = colorbar;
c.Ticks = [0.25 0.75];
c.TickLabels = {'OFF', 'ON'};