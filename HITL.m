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

global stopFlag; % Imergency Stop
global U_binary_global;
stopFlag = false;
U_binary_global = zeros(8,1);

prevTime = tic; % timer setting

real_time = 0;

% initialize
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
P_ukf = []; % 임시
mpc_computation_time_hist =[];
predicted_trajectories_hist = [];

final_cmd = [ 0, 0, 0, 0, 0, 0, 0, 0, 0 ]';
X = [ 400; 400; 0; 0; 0; 0 ];

% Jetson Orin Nano set
hwobj = params.System.hwjetson;

% socket communication
try
    client = tcpclient(params.System.host, params.System.commandPort, 'Timeout', 30);
    disp('Connected to Jetson server');
catch ME
    error('Failed to connect to Jetson server: %s', ME.message);
end

% UI Generate
createUI();
createSTOP_UI();

sendMotorCommand( client, 4 );
sendMotorCommand( client, 40 );
sendMotorCommand( client, 49 );
disp(1)
pause(1)
sendMotorCommand( client, 49 );
disp(2)
pause(1)
sendMotorCommand( client, 49 );
disp(3)
pause(1)
sendMotorCommand( client, 49 );
disp(4)
pause(1)
sendMotorCommand( client, 49 );
disp(5)
pause(1)
sendMotorCommand( client, 49 );
disp('It is warm')

while true

    tic;

    [ X_target, msp ] = generate_commands_wp( real_time, X );
    % [ X_target, msp ] = generate_commands_wpn( real_time );


    % Imergency stop

    if stopFlag
        sendRelayCommand(client, [ 1, 1, 1, 1, 1, 1, 1, 1 ]);
        sendRelayCommand(client, [ 1, 1, 1, 1, 1, 1, 1, 1 ]);
        sendRelayCommand(client, [ 1, 1, 1, 1, 1, 1, 1, 1 ]);
        sendRelayCommand(client, [ 1, 1, 1, 1, 1, 1, 1, 1 ]);
        sendRelayCommand(client, [ 1, 1, 1, 1, 1, 1, 1, 1 ]);
        sendRelayCommand(client, [ 1, 1, 1, 1, 1, 1, 1, 1 ]);
        sendRelayCommand(client, [ 1, 1, 1, 1, 1, 1, 1, 1 ]);
        sendRelayCommand(client, [ 1, 1, 1, 1, 1, 1, 1, 1 ]);
        sendRelayCommand(client, [ 1, 1, 1, 1, 1, 1, 1, 1 ]);
        sendRelayCommand(client, [ 1, 1, 1, 1, 1, 1, 1, 1 ]);
        sendMotorCommand( client, 1 );
        sendMotorCommand( client, 1 );
        sendMotorCommand( client, 1 ); 
        sendMotorCommand( client, 1 );
        sendMotorCommand( client, 1 );
        sendMotorCommand( client, 1 );
        sendMotorCommand( client, 1 );
        sendMotorCommand( client, 1 );
        sendMotorCommand( client, 1 );
        sendMotorCommand( client, 1 );
        break;
    end

    % Data reacive & process
    [ X, P, total_latency, interval, raw_data, X_Predict ] = processDataFromJetson_SUC( client, X, final_cmd, params, dt, prevTime );
    
    % Rx - time check
    prevTime = tic;  % Reset the timer

    U = Controller_PID_Argument( X, X_target, dt, params );

    [ final_cmd, debug_cmd_delta ] = Control_Allocator( U, params );

    % if X(1) >= 2173
    % 
    %     sendRelayCommand(client, [ 1, 1, 1, 1, 1, 1, 1, 1 ]);
    %     sendRelayCommand(client, [ 1, 1, 1, 1, 1, 1, 1, 1 ]);
    %     sendRelayCommand(client, [ 1, 1, 1, 1, 1, 1, 1, 1 ]);
    %     sendRelayCommand(client, [ 1, 1, 1, 1, 1, 1, 1, 1 ]);
    %     sendRelayCommand(client, [ 1, 1, 1, 1, 1, 1, 1, 1 ]);
    %     sendRelayCommand(client, [ 1, 1, 1, 1, 1, 1, 1, 1 ]);
    % 
    % else
        % Command_binary
        U_binary_global = final_cmd(1:8);
        RU = [ 1, 1, 1, 1, 1, 1, 1, 1 ]' - final_cmd(1:8);
        U_RW = final_cmd(9);
    
        sendRelayCommand( client, RU );
        sendMotorCommand( client, U_RW );
    % end
    state_history = [ state_history, X ];
    X_target_history = [X_target_history, X_target];  
    X_raw = [ X_raw, raw_data ];

    debug_control = [ debug_control, final_cmd(1:8) ];


    t = [ t, real_time ];

    dt = toc;
    % disp(dt)
    real_time = real_time + dt;

end

numsteps = numel(t)


%% Plot the results
figure(3);

subplot(3, 1, 1);
plot( t, state_history(1, :), '-b' , t, X_raw(1, :), '--g' , t, X_target_history(1, :), '--r' );
xlabel('[sec]') 
ylabel('[mm]')
title('X');
legend('X', 'Raw X', 'Target');
hold on


subplot(3, 1, 2);
plot( t, state_history(2, :), '-b' , t, X_raw(2, :), '--g' , t, X_target_history(2, :), '--r' );
xlabel('[sec]') 
ylabel('[mm]')
title('Y');
legend('Y', 'Raw Y', 'Target');
hold on


subplot(3, 1, 3);
plot( t, rad2deg(state_history(5, :)), '-b' , t, rad2deg(X_raw(6, :)), '--g' , t, rad2deg(X_target_history(5, :)), '--r' );
xlabel('[sec]') 
ylabel('[deg]')
title('psi');
legend('psi', 'Raw psi', 'Target');
hold on


figure(4);
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


figure(5);
plot(state_history(1,:), state_history(2,:), 'b-', 'LineWidth', 2); hold on;
% plot(X_target_history(1,:), X_target_history(2,:), 'ro', 'LineWidth', 2); % 웨이포인트 표시
xlabel('X [mm]');
ylabel('Y [mm]');
title('XY Trajectory');
legend('Actual Path', 'Target Path');
axis equal;
grid on;

% 오늘 날짜와 시간 정보를 사용하여 폴더 생성
timestamp = datestr(now,'yyyymmdd_HHMMSS');
save_folder = fullfile(pwd, ['result_' timestamp]);
if ~exist(save_folder, 'dir')
    mkdir(save_folder);
end

% 데이터 저장
data_filename = fullfile(save_folder, ['data_' timestamp '.mat']);
save(data_filename, 't', 'state_history', 'X_target_history', 'X_raw', 'debug_control');

% 모든 figure 저장
figList = findall(0, 'Type', 'figure');
for i = 1:length(figList)
    fig = figList(i);

    fig_filename_png = fullfile(save_folder, ['figure' num2str(fig.Number) '_' timestamp '.png']);
    fig_filename_fig = fullfile(save_folder, ['figure' num2str(fig.Number) '_' timestamp '.fig']);

    % uifigure 체크
    if isa(fig, 'matlab.ui.Figure')
        % uifigure의 경우 exportgraphics 대신 getframe 사용
        frame = getframe(fig); % figure 전체를 캡처
        imwrite(frame.cdata, fig_filename_png);

        % fig 파일로 저장 (uifigure도 fig로 저장 가능)
        savefig(fig, fig_filename_fig);
    else
        % 일반 figure일 경우 기존 방식 사용
        saveas(fig, fig_filename_png);
        savefig(fig, fig_filename_fig);
    end
end

disp(['모든 데이터와 그래프가 ' save_folder ' 폴더에 저장되었습니다.']);

