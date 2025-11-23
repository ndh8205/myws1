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
addpath(genpath('C:\Users\USER\Desktop\sat_hw_ver1'));

%% Initialize setting
params = Params_init(); % Load Parameter struct

% Simulation setting  
simulationTimeStep = 0.0625; % [sec]
dt = simulationTimeStep;
simulationEndTime = 60; % [sec]
numSteps = ceil( simulationEndTime / simulationTimeStep );

% Simulation setting
Main_Hz = params.System.MainLoopHz;
Sensor_VI_Hz = Main_Hz;
PID_control_Hz = Main_Hz;

U = zeros( 8,1 ); % ( T1 T2 T3 T4 T5 T6 T7 T8 RW )

Rk = params.EKF.R;

Q_Fx = 0.1;
Q_Fy = 0.1;
Q_Tau = 0.1;
Qw = diag( [ Q_Fx.^2, Q_Fx.^2, Q_Tau.^2 ]' );

% State Initialize
X = [ 0; 0; 0; 0; 0; 0 ]; % (posX, posY, V_X, V_Y, rotZ, rateZ)
XF = zeros( 12,1 );

% Time duty initialize
real_time = 0;

% Data log initialize (메모리 사전 할당)
state_history = zeros(6, numSteps);
state_history_ss = zeros(6, numSteps); % State Space 결과 저장
debug_control = zeros(9, numSteps);
t = zeros(1, numSteps);
target_history = zeros(6, numSteps);
debug_cmd_delta_hist = zeros(9, numSteps);
mpc_computation_time = zeros(1, numSteps);
predicted_trajectories = cell(1, numSteps);

% Main simulation loop
for i = 1:numSteps
    
    X_target = generate_commands(real_time); % Generate - X_target & tolerance (목표값, 허용오차)

    z = X + randn( 6, 1 ) .* [ Rk(1,1), Rk(2,2), Rk(3,3), Rk(4,4), Rk(5,5), Rk(6,6) ]';
    % [ X, P_ekf, X_raw, X_predict ] = processDataSIM( z, X, U, params, Sensor_VI_Hz );
    [ U , computation_time, predicted_trajectory] = Controller_NMPC_KJC2(X, X_target, dt, params, XF);

    disp(U)
    % 추가 부분
    X_old = X;

    % Motor_cmd = U( 9 );

    % % Scale the delta_w_rw to motor command (0-100, with 40 as idle)
    % min_input = -0.001; % Define minimum input based on your application
    % max_input = 0.001; % Define maximum input based on your application
    % 
    % motor_command = scale_command( Motor_cmd, min_input, max_input, 49 );
    % % Motor_mask = LIMIT2( motor_command, 0, 98 );
    % U( 9 ) = LIMIT2( motor_command, 0, 98 )


    % RK4 시뮬레이션
    X = srk4( @vehicle_dynamics_Airbearing_stochastic, X, U, Qw, dt, params, dt );

    % 추가 부분
    dx = X - X_old;
    y = X;
    XF = [dx; y];
    %

    disp("num : ")
    disp(i)
     
    % 결과 저장
    state_history(:, i) = X;
    debug_control(:, i) = U;
    t(i) = real_time;
    target_history(:, i) = X_target;
    mpc_computation_time(i) = computation_time;
    predicted_trajectories{i} = predicted_trajectory;

    real_time = real_time + simulationTimeStep;
    
end

%% Plot the results

figure(3);

subplot(3, 1, 1);
plot(t, state_history(1, :), 'b', t, target_history(1, :), 'r--');
hold on
xlabel('[sec]')
ylabel('[mm]')
title('X Position');
legend('RK4', 'Target');

subplot(3, 1, 2);
plot(t, state_history(2, :), 'b', t, target_history(2, :), 'r--');
hold on
xlabel('[sec]')
ylabel('[mm]')
title('Y Position');
legend('RK4', 'Target');

subplot(3, 1, 3);
plot(t, rad2deg(state_history(5, :)), 'b', t, rad2deg(target_history(5, :)), 'r--');
hold on
xlabel('[sec]')
ylabel('[deg]')
title('Yaw Angle (psi)');
legend('RK4', 'Target');

figure(4);

imagesc(t, 1:8, debug_control(1:8,:));
colormap([92, 136, 196; 255, 253, 181] / 255);
xlabel('Time [sec]');
ylabel('Relay Number');
title('Relay States Heatmap');
yticks(1:8);
grid on;
c = colorbar;
c.Ticks = [0.25 0.75];
c.TickLabels = {'OFF', 'ON'};
% 
% 
% % 추가적인 그래프 (MPC 계산 시간)
% figure(5);
% plot(t, mpc_computation_time);
% xlabel('Time [sec]');
% ylabel('Computation Time [sec]');
% title('MPC Computation Time');

% % 예측 궤적 플로팅 (매 10번째 스텝마다)
% figure(6);
% plot(state_history(1,:), state_history(2,:), 'b', 'LineWidth', 2);
% hold on;
% plot(target_history(1,:), target_history(2,:), 'r--', 'LineWidth', 2);
% for i = 1:10:numSteps
%     pred_traj = predicted_trajectories{i};
%     plot(pred_traj(1,:), pred_traj(2,:), 'g-');
% end
% xlabel('X Position [mm]');
% ylabel('Y Position [mm]');
% title('Actual Trajectory, Target, and Predicted Trajectories');
% legend('Actual', 'Target', 'Predicted');

figure(6);
plot(state_history(1,:), state_history(2,:), 'b', 'LineWidth', 2);
hold on;
for i = 1:10:numSteps
    pred_traj = predicted_trajectories{i};
    plot(pred_traj(1,:), pred_traj(2,:), 'g-');
end
xlabel('X Position [mm]');
ylabel('Y Position [mm]');
title('Actual Trajectory, Target, and Predicted Trajectories');
legend('Actual', 'Target', 'Predicted');