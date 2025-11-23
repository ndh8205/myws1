% Main Script
% This Code is HITL (Hardware in the loop)
% Ver.2.0.0 (only PID)
% 2024-08-12 / Controla Project
% Made by NDH & LJC

% Initialize Matlab
clc;
clear all;
close all;

% Add Library directory
addpath(genpath('C:\Users\DDHD\Desktop\shj\main'));

% Initialize setting
params = Params_init(); % Load Parameter struct

global stopFlag; % Emergency Stop
global U_binary_global;
stopFlag = false;
U_binary_global = zeros(8,1);

prevTime = tic; % Timer setting

real_time = 0;

% Initialize
t = [];
dt = 1 / params.System.MainLoopHz;
intervals = [];
X_raw = [];
state_history = [];
X_target_history = [];
X_Predict_EKF_hist = [];

debug_FT = [];
debug_control = [];
debug_cmd_delta_hist = [];
debug_motor_cmd = [];

debug3sigma = [];
temp3sigma_EKF = [];
temp3sigma_UKF = [];
P_ukf = []; % Temporary
mpc_computation_time_hist =[];
predicted_trajectories_hist = [];

final_cmd = [ 0, 0, 0, 0, 0, 0, 0, 0, 0 ]';
X = [ 2074; -307; 0; 0; 0; 0 ];

% Jetson Orin Nano set
hwobj = params.System.hwjetson;

% Socket communication
try
    client = tcpclient(params.System.host, params.System.commandPort, 'Timeout', 30);
    disp('Connected to Jetson server');
catch ME
    error('Failed to connect to Jetson server: %s', ME.message);
end

% UI Generate
createUI();
createSTOP_UI();

% 메인 루프 시작 전 짧은 대기
pause(0.5);

% 추가 데이터 로그 변수 생성
msp_history = zeros(1, 1e6); % 모드 상태 기록 (미리 큰 크기로 초기화)
alignment_point = []; % 자세 정렬 시작 지점 기록
approach_point_history = []; % 접근 웨이포인트 기록

% Main simulation loop
step = 1;
while true

    tic;

    % Command Generator: 현재 시간과 상태만 입력으로 받음
    [X_target, msp, approach_point] = generate_commands_shj(real_time, X);

    msp_history(step) = msp; % 모드 상태 기록

    % 모드 전환 시점 기록 (정렬 시작 지점)
    if step > 1 && msp_history(step-1) ~= msp_history(step) && msp == 2
        alignment_point = [alignment_point; X(1:2)'];
    end

    % 도킹 완료 시 루프 종료
    if msp == 3
        disp('Docking completed successfully.');
        break;
    end

    % Emergency stop
    if stopFlag
        relay_commands = ones(8,1); % 모든 릴레이를 ON 상태로 설정
        for relay_step = 1:10
            sendRelayCommand(client, relay_commands);
        end
        sendMotorCommand(client, 1); % 모든 모터를 정지 명령
        disp('Emergency stop activated.');
        break;
    end

    % Data receive & process
    [X, P, total_latency, interval, raw_data, X_Predict] = processDataFromJetson_SUC2(client, X, final_cmd, params, dt, prevTime);

    % Rx - time check
    prevTime = tic;  % Reset the timer

    % PID Controller
    U = Controller_PID_Argument(X, X_target, dt, params);

    % Control Allocator
    [final_cmd, debug_cmd_delta] = Control_Allocator(U, params);

    % Command_binary
    U_binary_global = final_cmd(1:8);
    RU = ones(8,1) - final_cmd(1:8); % Relay ON/OFF 상태 계산
    U_RW = final_cmd(9);

    sendRelayCommand(client, RU);
    sendMotorCommand(client, U_RW);

    % Data logging
    state_history = [state_history, X];
    X_target_history = [X_target_history, X_target];
    X_raw = [X_raw, raw_data];

    debug_control = [debug_control, final_cmd(1:8)];

    t = [t, real_time];

    dt = toc;
    real_time = real_time + dt;

    step = step + 1;

    % Prevent memory overflow
    if step > size(msp_history,2)
        msp_history = [msp_history, zeros(1,1e6)];
    end

end

numsteps = numel(t);
disp(['Number of steps: ', num2str(numsteps)]);

%% Plot the results
figure(10);
hold on;
% 2D Trajectory Plot
plot(state_history(1, :), state_history(2, :), 'b', 'LineWidth', 2); % Vehicle Path
plot(X_target_history(1, :), X_target_history(2, :), 'r--', 'LineWidth', 1.5); % Target Path

% 도킹 지점 표시
x_dock = 2800;
y_dock = 1500;
plot(x_dock, y_dock, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
text(x_dock + 50, y_dock, 'Docking Point');

% 접근 웨이포인트 표시 (첫 번째 접근 포인트)
if ~isempty(X_target_history)
    approach_point = X_target_history(1:2,1);
    plot(approach_point(1), approach_point(2), 'gs', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    text(approach_point(1) + 50, approach_point(2), 'Approach Point');
end

% 자세 정렬 시작 지점 표시
if ~isempty(alignment_point)
    plot(alignment_point(:,1), alignment_point(:,2), 'md', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
    text(alignment_point(:,1) + 50, alignment_point(:,2), 'Alignment Point');
end

% 입사각 방향 표시
theta_dock = deg2rad(5);
quiver(x_dock, y_dock, ...
    200 * cos(theta_dock), 200 * sin(theta_dock), ...
    'Color', 'k', 'LineWidth', 2, 'MaxHeadSize', 5);
text(x_dock + 200 * cos(theta_dock) + 50, ...
    y_dock + 200 * sin(theta_dock), 'Docking Angle');

xlabel('X Position [mm]');
ylabel('Y Position [mm]');
title('2D Trajectory of the Vehicle');
legend('Vehicle Path', 'Target Path', 'Docking Point', 'Approach Point', 'Alignment Point', 'Docking Angle');
grid on;
axis equal;
hold off;

% 기존의 상태 플롯 유지
figure(2);
subplot(3, 1, 1);
plot(t, state_history(1, :), '-b', t, X_raw(1, :), '--g', t, X_target_history(3, :), '--r');
xlabel('[sec]');
ylabel('[mm]');
title('V_X Position');
legend('x_X', 'Raw x_X', 'Target x_X');
hold on;

subplot(3, 1, 2);
plot(t, state_history(2, :), '-b', t, X_raw(2, :), '--g', t, X_target_history(4, :), '--r');
xlabel('[sec]');
ylabel('[mm]');
title('V_Y Position');
legend('x_Y', 'Raw x_Y', 'Target x_Y');
hold on;

subplot(3, 1, 3);
plot(t, rad2deg(state_history(5, :)), '-b', t, rad2deg(X_raw(6, :)), '--g', t, rad2deg(X_target_history(5, :)), '--r');
xlabel('[sec]');
ylabel('[deg]');
title('Yaw Angle (psi)');
legend('Yaw Angle', 'Raw Yaw Angle', 'Target Yaw Angle');
hold on;

% Relay States Heatmap
figure(5);
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
