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
addpath(genpath('C:\Users\DDHD\Desktop\sat_hw_ver3'));

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



U_binary = [ 0, 0, 0, 0, 0, 0, 0 ]';
X = [ 0; 0; 0; 0; 0; 0 ];

% Jetson Orin Nano set
hwobj = params.System.hwjetson;

% socket communication
client = tcpclient( params.System.host, params.System.commandPort );

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

% UI Generate
createUI();
createSTOP_UI();

while true

    tic;

    [ X_target, tolerance ] = generate_commands( real_time );

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
    [ X, P_ekf, total_latency, interval, raw_data, X_Predict ] = processDataFromJetson3( client, X, U_binary, params, dt, prevTime );

    % Rx - time check
    prevTime = tic;  % Reset the timer

    % U = Controller_PID_Argument( X, X_target, dt, params );
    % [ final_cmd, debug_cmd_delta ] = Control_Allocator( U, params );
    [ final_cmd , computation_time, predicted_trajectory ] = Controller_NMPC(X, X_target, dt, params);

    % Command_binary
    % Relay_mask = ;
    U_binary = final_cmd(1:8) > mean( final_cmd( 1:8 ) );
    U_binary_global = U_binary;
    

    RU = [ 1, 1, 1, 1, 1, 1, 1, 1 ]' - U_binary;
    % U_RW = final_cmd(9);

    % disp( zeros(8,1) - RU)

    sendRelayCommand( client, RU );
    % sendMotorCommand( client, U_RW );

    debug_FT = [ debug_FT, final_cmd];
    % debug_cmd_delta_hist = [ debug_cmd_delta_hist, debug_cmd_delta ];
    debug_control = [ debug_control, U_binary ];

    state_history = [ state_history, X ];
    X_target_history = [X_target_history, X_target];  
    X_raw = [ X_raw, raw_data ];
    X_Predict_EKF_hist = [ X_Predict_EKF_hist, X_Predict ];

    temp3sigma_EKF = [ temp3sigma_EKF , 3*sqrt(diag(P_ekf))];
    temp3sigma_UKF = [ temp3sigma_UKF , 3*sqrt(diag(P_ukf))];

    mpc_computation_time_hist =[ mpc_computation_time_hist, computation_time ];
    predicted_trajectories_hist = [ predicted_trajectories_hist, predicted_trajectory ];

    t = [ t, real_time ];

    dt = toc;
    real_time = real_time + dt;

end

numsteps = numel(t)


%% Plot the results
figure(3);

subplot(3, 1, 1);
plot( t, state_history(1, :), '-b' , t, X_raw(1, :), '--g' , t, X_target_history(1, :), '--r' , t, X_Predict_EKF_hist(1, :), '--m' );
xlabel('[sec]') 
ylabel('[mm]')
title('X');
legend('X', 'Raw X', 'Target');
hold on


subplot(3, 1, 2);
plot( t, state_history(2, :), '-b' , t, X_raw(2, :), '--g' , t, X_target_history(2, :), '--r' , t, X_Predict_EKF_hist(2, :), '--m' );
xlabel('[sec]') 
ylabel('[mm]')
title('Y');
legend('Y', 'Raw Y', 'Target');
hold on


subplot(3, 1, 3);
plot( t, rad2deg(state_history(5, :)), '-b' , t, rad2deg(X_raw(5, :)), '--g' , t, rad2deg(X_target_history(5, :)), '--r' , t, X_Predict_EKF_hist(5, :), '--m' );
xlabel('[sec]') 
ylabel('[deg]')
title('psi');
legend('psi', 'Raw psi', 'Target');
hold on

% % vector save1
% figureFilename1 = fullfile(saveDir, ['figure1_' timestamp '.pdf']);
% print(figure(3), '-dpdf', figureFilename1);

figure(4);

subplot(3, 1, 1);
plot( t, debug_FT(1, :), '-b' );
xlabel('[sec]') 
ylabel('[mN]')
title('F_x');
legend('F_x');
hold on


subplot(3, 1, 2);
plot( t, debug_FT(2, :), '-b' );
xlabel('[sec]') 
ylabel('[mN]')
title('F_y');
legend('F_y');
hold on


subplot(3, 1, 3);
plot( t, debug_FT(3, :), '-b' );
xlabel('[sec]') 
ylabel('[mNmm]')
title('Tau');
legend('Tau');
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


% figure(6);
% 
% for i = 1:8
%     plot(t, debug_cmd_delta_hist(i, :) )
%     ylabel('Thrust (mN)');
%     grid on
%     hold on
% end
% 
% legend('T1', 'T2', 'T3', 'T4', 'T5', 'T6', 'T7', 'T8');

% % vector save2
% figureFilename2 = fullfile(saveDir, ['figure2_' timestamp '.pdf']);
% print(figure(4), '-dpdf', figureFilename2);

% figure(7);
% 
% subplot(3, 1, 1);
% plot(t, debug_motor_cmd(1, :));
% xlabel('[sec]') 
% ylabel('[cmd tau]')
% title('cmd tau rw');
% hold on
% 
% subplot(3, 1, 2);
% plot(t, debug_motor_cmd(2, :));
% xlabel('[sec]') 
% ylabel('[delta w rw]')
% title('delta w rw');
% hold on
% 
% subplot(3, 1, 3);
% plot(t, debug_motor_cmd(3, :));
% xlabel('[sec]') 
% ylabel('[motor command]')
% title('motor command');
% hold on

% % vector save3
% figureFilename3 = fullfile(saveDir, ['figure3_' timestamp '.pdf']);
% print(figure(5), '-dpdf', figureFilename3);


figure(8);

caxis = 1;

subplot(3,1,caxis)
plot(t,temp3sigma_EKF( caxis, : ),'r-');
title('Position Error, Estimate')
hold on;grid on ;
plot(t,X_raw( caxis, : ) - state_history( caxis, : ),'b');
hold on;grid on ;
plot(t,-temp3sigma_EKF( caxis, : ),"r-");
grid on ;
hold on;
xlabel('Time(sec)');ylabel('X error [mm]');
ylim([-max(temp3sigma_EKF( caxis, : ))*2 max(temp3sigma_EKF( caxis, : ))*2 ]);

caxis = 2;

subplot(3,1,caxis)
plot(t,temp3sigma_EKF( caxis, : ),'r-');
title('Position Error, Estimate')
hold on;grid on ;
plot(t,X_raw( caxis, : ) - state_history( caxis, : ),'b');
hold on;grid on ;
plot(t,-temp3sigma_EKF( caxis, : ),"r-");
grid on ;
hold on;
xlabel('Time(sec)');ylabel('Y error [mm]');
ylim([-max(temp3sigma_EKF( caxis, : ))*2 max(temp3sigma_EKF( caxis, : ))*2 ]);

caxis = 5;

subplot(3,1,3)
plot(t,temp3sigma_EKF( caxis, : ),'r-');
title('Attitude Error , Estimate')
hold on;grid on ;
plot(t,X_raw( caxis, : ) - state_history( caxis, : ),'b');
hold on;grid on ;
plot(t,-temp3sigma_EKF( caxis, : ),"r-");
grid on ;
hold on;
xlabel('Time(sec)');ylabel('psi error [deg]');
ylim([-max(temp3sigma_EKF( caxis, : ))*2 max(temp3sigma_EKF( caxis, : ))*2 ]);

% % vector save4
% figureFilename4 = fullfile(saveDir, ['figure4_' timestamp '.pdf']);
% print( figure(6), '-dpdf', figureFilename4 );


% 추가적인 그래프 (MPC 계산 시간)
figure(9);
plot(t, mpc_computation_time_hist);
xlabel('Time [sec]');
ylabel('Computation Time [sec]');
title('MPC Computation Time');

% 예측 궤적 플로팅 (매 10번째 스텝마다)
figure(10);
plot(state_history(1,:), state_history(2,:), 'b', 'LineWidth', 2);
hold on;
plot(X_target_history(1,:), X_target_history(2,:), 'r--', 'LineWidth', 2);
for i = 1:50:numsteps
    pred_traj = predicted_trajectories_hist(:,i);
    plot(pred_traj(1,:), pred_traj(2,:), 'g-');
end
xlabel('X Position [mm]');
ylabel('Y Position [mm]');
title('Actual Trajectory, Target, and Predicted Trajectories');
legend('Actual', 'Target', 'Predicted');