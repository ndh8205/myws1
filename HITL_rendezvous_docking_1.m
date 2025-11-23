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
addpath(genpath('C:\Users\DDHD\Desktop\sat_hw_ver_Nonlinear\main'));

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
X = [  2074; -307; 0; 0; 0; 0 ];

% Jetson Orin Nano set
hwobj = params.System.hwjetson;

% socket communication
client = tcpclient( params.System.host, params.System.commandPort );

% UI Generate
createUI();
createSTOP_UI();
% 
% sendMotorCommand( client, 4 );
% sendMotorCommand( client, 40 );
% sendMotorCommand( client, 49 );
% disp(1)
% pause(1)
% sendMotorCommand( client, 49 );
% disp(2)
% pause(1)
% sendMotorCommand( client, 49 );
% disp(3)
% pause(1)
% sendMotorCommand( client, 49 );
% disp(4)
% pause(1)
% sendMotorCommand( client, 49 );
% disp(5)
% pause(1)
% sendMotorCommand( client, 49 );
% disp('It is warm')



while true

    tic;

    [ X_target, msp ] = generate_commands( real_time );


        % FOV 체크 및 명령 생성
    if ~check_fov(X, X_target)  % FOV 체크 함수의 인자를 X_target으로 수정
        % FOV 밖에 있을 때: 마스터 포인트 접근
        [chaser_cmd, ~, msp] = generate_commands_rendezvous2(X, X_target);
        % disp('out fov')
    else
        % FOV 안에 있을 때: 도킹 명령 (중심선 접근 및 도킹)
        [chaser_cmd, ~, msp] = generate_commands_dock(real_time, X, X_target);
        % disp('in fov')
    end
    % disp(dt)
    
    % disp('cmd :')
    % disp(chaser_cmd(2))
    % disp(chaser_cmd(3))
    % disp(rad2deg(chaser_cmd(5)))
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
    [ X, P, total_latency, interval, raw_data, X_Predict ] = EKF_multi_1( client, X, final_cmd, params, dt, prevTime );

    % Rx - time check
    prevTime = tic;  % Reset the timer

    U = Controller_PID_Argument( X, chaser_cmd, dt, params );
    [ final_cmd, debug_cmd_delta ] = Control_Allocator( U, params );

    % Command_binary
    U_binary_global = final_cmd(1:8);
    RU = [ 1, 1, 1, 1, 1, 1, 1, 1 ]' - final_cmd(1:8);
    U_RW = final_cmd(9);
    

    nocontrol = [ 1, 1, 1, 1, 1, 1, 1, 1 ]';
    sendRelayCommand( client, nocontrol );
    sendMotorCommand( client, 0 );

    state_history = [ state_history, X ];
    X_target_history = [X_target_history, X_target];  
    X_raw = [ X_raw, raw_data ];


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


