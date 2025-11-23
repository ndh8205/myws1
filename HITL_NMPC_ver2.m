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
debug_control = [];
debug_control2 = [];


final_cmd = [ 0, 0, 0, 0, 0, 0, 0, 0, 0 ]';
X = [ 517; 484; 0; 0; 0; 0 ];
 
X_old = [ 517; 484; 0; 0; 0; 0 ]; % NEED MPC
y = [ 517; 484; 0; 0; 0; 0 ]; % NEED MPC

XF = [ X; y ];  % NEED MPC


% Jetson Orin Nano set
hwobj = params.System.hwjetson;

% socket communication
client = tcpclient( params.System.host, params.System.commandPort );

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

idle = 49; % NEED MPC
omega_limit = params.Reaction_wheel.omega_limit; % NEED MPC
I_wheel = params.Reaction_wheel.I_wheel; % NEED MPC
MAA = omega_limit / dt; % NEED MPC
max_torque = I_wheel/2 * MAA; % NEED MPC

while true

    tic;

    [ X_target, msp ] = generate_commands( real_time );
    % disp('X_target :')
    % disp(X_target)

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
    [ X, P_ekf, total_latency, interval, raw_data, X_Predict ] = processDataFromJetson_SUC2( client, X, final_cmd, params, dt, prevTime );

    % Rx - time check
    prevTime = tic;  % Reset the timer
    
    [ U , computation_time, predicted_trajectory] = Controller_NMPC_KJC_Nonterminal_v2(X, X_target, 0, max_torque, idle, dt, params, XF);
    % [U, computation_time, predicted_trajectory] = Controller_NMPC_ver1(X, X_target, 0, max_torque, idle, dt, params, XF);
    % debug_control2 = [ debug_control2, U(9) ];
    [ final_cmd, debug_cmd_delta ] = Control_Allocator_MPC( U, params );

    % Command_binary
    U_binary_global = final_cmd(1:8);
    RU = [ 1, 1, 1, 1, 1, 1, 1, 1 ]' - final_cmd(1:8);
    U_RW = final_cmd(9);

    sendRelayCommand( client, RU );
    sendMotorCommand( client, U_RW );

    dx = X - X_old;
    y = X;
    XF = [dx; y];

    X_old = X;

    debug_control = [ debug_control, final_cmd(1:8) ];
    
    state_history = [ state_history, X ];
    X_target_history = [X_target_history, X_target];  
    X_raw = [ X_raw, raw_data ];
    X_Predict_EKF_hist = [ X_Predict_EKF_hist, X_Predict ];

    t = [ t, real_time ];

    dt = toc;
    real_time = real_time + dt;

    disp('real_time:')
    disp(real_time)

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
plot( t, rad2deg(state_history(5, :)), '-b' , t, rad2deg(X_raw(6, :)), '--g' , t, rad2deg(X_target_history(5, :)), '--r');
xlabel('[sec]') 
ylabel('[deg]')
title('psi');
legend('psi', 'Raw psi', 'Target');
hold on


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
