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

%% Initialize setting
params = Params_init(); % Load Parameter struct

Xinit = [ 517; 484; 0; 0; 0; 0 ]; % ( posX, posY, V_X, V_Y, rotZ, rateZ )
X = Xinit;
Master_point = [ 2400; 2400; 0; 0; 0; 0 ]; % Initialize Mothership position and orientation
X_AB = Xinit;
X_linear = Xinit; % 선형 모델 초기 상태
Xd = Xinit;
xl = Xinit;
Xdn = Xinit;
Xdm = Xinit;


Rk = params.EKF.Rm;

Q_Fx = 0;
Q_Fy = 0;
Q_Tau = 0;
Qw = diag( [ Q_Fx.^2, Q_Fx.^2, Q_Tau.^2 ]' );

Iz = params.Airbearing.I_Z;
D = params.Airbearing.D_ref;
M = params.Airbearing.m;
T = params.Airbearing.Thrust;

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
state_history_AB = zeros(6, sim_step);
state_history_dist = zeros(6, sim_step);

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
motor_cmd_hist = zeros(1, sim_step);
state_history_linear = zeros(6, sim_step);
Force_history_srk4 = zeros(3, sim_step);
Force_history_AB = zeros(3, sim_step);


% Main simulation loop
for i = 1 : sim_step
    
    X_target = generate_commands(real_time); % Generate - X_target & tolerance


    if mod(i-1, PID_control_interval) == 0 

        U = Controller_PID_Argument( X, X_target, sim_dt, params );

    end

    [ final_cmd, debug_cmd_delta ]  = Control_Allocator( U, params );

    final_cmd(9) = cmd2tq_origin( final_cmd(9), 49, params, sim_dt );


    % Stochastic RK4 simulation
    [ X, Fskr4 ] = srk4_debug( @vehicle_dynamics_Airbearing_stochastic_debug, X, final_cmd, Qw, sim_dt, params, sim_dt );
    [ X_AB, FAB ] = rk4_debug( @vehicle_dynamics_Airbearing_AXBU, X_AB, final_cmd, sim_dt, params );
    FAB = [ FAB(3); FAB(4); FAB(6) ];


    FJ =  [
            1, 0, sim_dt*cos(Xdn(5)), -sim_dt*sin(Xdn(5)),   -sim_dt * ( Xdn(3)*sin(Xdn(5)) + Xdn(4)*cos(Xdn(5)) ),           0;
            0, 1, sim_dt*sin(Xdn(5)),  sim_dt*cos(Xdn(5)),   sim_dt * (  Xdn(3)*cos(Xdn(5)) - Xdn(4)*sin(Xdn(5)) ),           0;
            0, 0,            1,      -sim_dt*Xdn(6),                                           0,  -sim_dt * Xdn(4);
            0, 0,      sim_dt*Xdn(6),             1,                                           0,   sim_dt * Xdn(3);
            0, 0,            0,             0,                                           1,          sim_dt;
            0, 0,            0,             0,                                           0,           1
         ];

    F =  [
        1, 0, sim_dt*cos(Xd(5)), -sim_dt*sin(Xd(5)),    0,           0;
        0, 1, sim_dt*sin(Xd(5)),  sim_dt*cos(Xd(5)),    0,           0;
        0, 0,                 1,      -sim_dt*Xd(6),    0,           0;
        0, 0,      sim_dt*Xd(6),                  1,    0,           0;
        0, 0,                 0,                  0,    1,      sim_dt;
        0, 0,                 0,                  0,    0,           1
     ];

    B = [
            0,          0,           0,          0,           0,          0,           0,          0       0;
            0,          0,           0,          0,           0,          0,           0,          0       0;
            0,       -T/M,        -T/M,          0,           0,        T/M,         T/M,          0       0;
         -T/M,          0,           0,        T/M,         T/M,          0,           0,       -T/M       0;
            0,          0,           0,          0,           0,          0,           0,          0       0;
        -(D*T)/Iz,   (D*T)/Iz,   -(D*T)/Iz,   (D*T)/Iz,   -(D*T)/Iz,   (D*T)/Iz,   -(D*T)/Iz,   (D*T)/Iz     1/Iz
        ] .* sim_dt;

    
    delta_Xd = FJ * (Xdn-Xdm) + B * final_cmd;
    Xdn = Xdm + delta_Xd;
    Xdm = delta_Xd;

    
    FABd = B*final_cmd;
    FABd = [ FABd(3); FABd(4); FABd(6) ];

    % data save
    state_history(:, i) = X;
    state_history_AB(:, i) = X_AB;
    state_history_linear(:, i) = Xdn;
    Masterpoint_history(:, i) = Master_point;
    motor_cmd_hist(:, i) = final_cmd(9);
    t(i) = real_time;
    target_history(:, i) = X_target;
    Force_history_srk4(:, i) = Fskr4;
    Force_history_AB(:, i) = FAB;
    
    real_time = real_time + sim_dt;
end

%% Plot the results 1

figure(1);
subplot(3, 1, 1);
plot(t, state_history(1, :), 'b', t, target_history(1, :), 'r--' );
hold on
plot(t, state_history_AB(1, :), 'm--' ,t, state_history_linear(1, :), 'k--')
% plot(t, state_history_AB(1, :), 'm--')


xlabel('[sec]')
ylabel('[mm]')
title('X Position');
% legend('Nonlinear', 'Target', 'Nonlinear matrix', 'Nonlinear discreat');
legend('Nonlinear', 'Target', 'linear', 'linear discreat');


subplot(3, 1, 2);
plot(t, state_history(2, :), 'b', t, target_history(2, :), 'r--' );
hold on
plot( t, state_history_AB(2, :), 'm--' ,t, state_history_linear(2, :), 'k--')
% plot( t, state_history_AB(2, :), 'm--')

xlabel('[sec]')
ylabel('[mm]')
title('Y Position');
% legend('Nonlinear', 'Target', 'Nonlinear matrix', 'Nonlinear discreat');
legend('Nonlinear', 'Target', 'linear', 'linear discreat');


subplot(3, 1, 3);
plot(t, rad2deg(state_history(5, :)), 'b', t, rad2deg(target_history(5, :)), 'r--');
hold on
plot(t, rad2deg(state_history_AB(5, :)), 'm--', t, rad2deg(state_history_linear(5, :)), 'k--')
% plot(t, rad2deg(state_history_AB(5, :)), 'm--')


xlabel('[sec]')
ylabel('[deg]')
title('Yaw Angle (psi)');
% legend('Nonlinear', 'Target', 'Nonlinear matrix', 'Nonlinear discreat');
legend('Nonlinear', 'Target', 'linear', 'linear discreat');


%% Plot the results 2

figure(2);
subplot(3, 1, 1);
plot(t, Force_history_srk4(1, :), 'b', t, Force_history_AB(1, :), 'r--' );
hold on

xlabel('[sec]')
ylabel('[mN]')
title('X F');
legend('SRK4', 'AB');
% legend('Nonlinear', 'Nonlinear matrix');


subplot(3, 1, 2);
plot(t, Force_history_srk4(2, :), 'b', t, Force_history_AB(2, :), 'r--' );
hold on

xlabel('[sec]')
ylabel('[mN]')
title('Y F');
legend('SRK4', 'AB');
% legend('Nonlinear', 'Nonlinear matrix');


subplot(3, 1, 3);
plot(t, Force_history_srk4(3, :), 'b', t, Force_history_AB(3, :), 'r--' );
hold on

xlabel('[sec]')
ylabel('[mmN]')
title('Torque');
legend('SRK4', 'AB');
% legend('Nonlinear', 'Nonlinear matrix');



