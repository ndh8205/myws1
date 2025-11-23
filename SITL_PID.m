clc;
clear all;
close all;

% Add Library directory
addpath(genpath('C:\Users\DDHD\Desktop\shj\main'));

% Initialize setting
params = Params_init(); % Load Parameter struct

X = [ 517; 484; 0; 0; 0; 0 ]; % ( posX, posY, V_X, V_Y, rotZ, rateZ )
master_point = [2400; 2400; 0; 0; deg2rad(90); 0]; % Initialize Mothership position and orientation
X_true = X;
Xk = X;
z = X; % ( posX, posY, V_X, V_Y, rotZ, rateZ )
final_cmd = [ zeros( 8,1 ); 49 ]; % ( T1 T2 T3 T4 T5 T6 T7 T8 RW )

P = params.EKF.P;
Rk = params.EKF.R;
Rm = params.EKF.Rm;

Q_Fx = 1;
Q_Fy = 1;
Q_Tau = 1;
Qw = diag( [ Q_Fx.^2, Q_Fx.^2, Q_Tau.^2 ]' );


% Simulation setting
Main_Hz = params.System.MainLoopHz;
Sensor_VI_Hz = params.Sensor.VICON_HZ;
PID_control_Hz = Main_Hz;
AHRS_HZ = params.Sensor.AHRS_HZ;
OPENCV_HZ = params.Sensor.OPENCV_HZ;

sim_dt = 1 / Main_Hz; % [sec]
sim_Time = 160; % [sec]
sim_step = ceil( sim_Time / sim_dt );

% Sensor & Control interval Setting
Sensor_VI_interval = Main_Hz / OPENCV_HZ; % sensor sampling set up
PID_control_interval = Main_Hz / PID_control_Hz; 
 
% Time duty initialize
real_time = 0;

% Data log initialize
state_history = zeros(6, sim_step);
t = zeros(1, sim_step);
target_history = zeros(6, sim_step);


% Main simulation loop
for i = 1 : sim_step

    tic;
    
    X_target = generate_commands( real_time, Xk );

    if mod(i-1, Sensor_VI_interval) == 0 

        [ z_AHRS, z_AHRS_true ] = measure_sensor_AHRS( Xk, master_point, Rm );
        [ z_Aruco, z_Aruco_true ] = measure_sensor_Aruco( Xk, master_point, Rm );
        
        z = [ z_Aruco; z_AHRS ];
        z_true = [ z_Aruco_true; z_AHRS_true ];

        [Xk, P, yhat, X_predict] = processDataSIM_Muti_EKF(z, P, Xk, master_point, final_cmd, params, sim_dt);

       % state filter history
        X_rel_I = master_point(1:2) - Xk(1:2);
        X_rel_B = [cos(Xk(5)), sin(Xk(5)); -sin(Xk(5)), cos(Xk(5))] * X_rel_I;
        rho_k = sqrt(X_rel_B(1)^2 + X_rel_B(2)^2);
        theta_k = atan2(X_rel_B(2), X_rel_B(1));
        r_k = Xk(6);

        % Measurement vector
        h1 = [ rho_k; theta_k; r_k ];

    end

    if mod(i-1, PID_control_interval) == 0 

        U = Controller_PID_Argument( Xk, X_target, sim_dt, params );

    end

    [ final_cmd, debug_cmd_delta ]  = Control_Allocator( U, params );
    
    % Stochastic RK4 simulation
    Xk = srk4( @vehicle_dynamics_Airbearing_stochastic, Xk, final_cmd, Qw, sim_dt, params, sim_dt );

    % data save
    state_history( :, i ) = Xk;
    t( i ) = real_time;
    target_history( :, i ) = X_target;
      
    real_time = real_time + sim_dt;

    sim_timer = toc;

end

%% Plot the results

figure(1);
subplot(3, 1, 1);
plot(t, state_history(1, :), 'b', t, target_history(1, :), 'r--');
hold on
xlabel('[sec]')
ylabel('[mm]')
title('X Position');
legend('EKF-Srk1 Estimate', 'Target' );

subplot(3, 1, 2);
plot(t, state_history(2, :), 'b', t, target_history(2, :), 'r--');
hold on
xlabel('[sec]')
ylabel('[mm]')
title('Y Position');
legend('EKF-Srk1 Estimate', 'Target' );

subplot(3, 1, 3);
plot(t, rad2deg(state_history(5, :)), 'b', t, rad2deg(target_history(5, :)), 'r--');
hold on
xlabel('[sec]')
ylabel('[deg]')
title('Yaw Angle (psi)');
legend('EKF-Srk1 Estimate', 'Target' );

%% 2D Trajectory Plot
figure(2);
hold on;
grid on;

% Plot trajectory
plot(state_history(1, :), state_history(2, :), 'b-', 'LineWidth', 1);

% 도킹 대상 위치
target_pos = [2400; 1500];  % 직접 위치 지정
scatter(target_pos(1), target_pos(2), 100, 'filled', 'y', 'MarkerEdgeColor', 'k');

% Plot pre-docking point (도킹 대상의 y축 방향으로)
R_pre = 800;  % pre-docking 거리
predocking_x = target_pos(1) - R_pre;  % 90도일 때는 단순히 R_pre만큼 왼쪽으로
predocking_y = target_pos(2);          % y좌표는 동일하게
scatter(predocking_x, predocking_y, 100, 'filled', 'k');

% 시작 위치 표시
scatter(state_history(1,1), state_history(2,1), 100, 'm', 'filled');

% 도킹 대상의 자세 표시 (화살표) - y축이 전진방향
arrow_length = 200;  % 화살표 길이
% 도킹 대상의 x축 방향 (빨간색)
quiver(target_pos(1), target_pos(2), ...
       arrow_length*cos(deg2rad(90)), arrow_length*sin(deg2rad(90)), ...
       'r', 'LineWidth', 2);
% 도킹 대상의 y축 방향 (초록색)
quiver(target_pos(1), target_pos(2), ...
       -arrow_length*sin(deg2rad(90)), arrow_length*cos(deg2rad(90)), ...
       'g', 'LineWidth', 2);

% 현재 위성의 마지막 위치와 자세 표시
last_pos = state_history(1:2, end);
last_attitude = state_history(5, end);
% 현재 위성의 x축 방향 (빨간색)
quiver(last_pos(1), last_pos(2), ...
       arrow_length*cos(last_attitude), arrow_length*sin(last_attitude), ...
       'r', 'LineWidth', 2);
% 현재 위성의 y축 방향 (초록색)
quiver(last_pos(1), last_pos(2), ...
       -arrow_length*sin(last_attitude), arrow_length*cos(last_attitude), ...
       'g', 'LineWidth', 2);

% Plot formatting
xlabel('X Position [mm]');
ylabel('Y Position [mm]');
title('2D Trajectory with Pre-docking Point');
legend('Trajectory', 'Target Satellite', 'Pre-docking Point', 'Start Position', ...
       'Target x-axis', 'Target y-axis', 'Current x-axis', 'Current y-axis', ...
       'Location', 'best');
axis equal;
xlim([0 3000]);
ylim([0 3000]);

% 각도 표시를 위한 텍스트
text_offset = 100;  % 텍스트 오프셋 거리
text(predocking_x + text_offset, predocking_y, ...
     ['\theta = ' num2str(90) '°'], ...
     'HorizontalAlignment', 'left');

% %% 애니메이션 생성 (Figure 3)
% 
% figure(3);
% hold on;
% grid on;
% axis equal;
% xlabel('X Position [mm]');
% ylabel('Y Position [mm]');
% title('2D Trajectory Animation');
% 
% % 플롯 범위 설정 (필요에 따라 조정)
% xlim([0 3000]);
% ylim([0 3000]);
% 
% % 도킹 대상 위치 및 사전 도킹 포인트 플롯
% scatter(target_pos(1), target_pos(2), 100, 'y', 'filled', 'MarkerEdgeColor', 'k');
% % scatter(predocking_x, predocking_y, 100, 'k', 'filled');
% 
% % 시작 위치 표시
% scatter(state_history(1,1), state_history(2,1), 100, 'm', 'filled');
% 
% % 도킹 대상의 자세 표시 (화살표) - y축이 전진 방향
% arrow_length = 200;  % 화살표 길이
% quiver_target_x = quiver(target_pos(1), target_pos(2), ...
%            arrow_length*cos(deg2rad(90)), arrow_length*sin(deg2rad(90)), ...
%            'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
% quiver_target_y = quiver(target_pos(1), target_pos(2), ...
%            -arrow_length*sin(deg2rad(90)), arrow_length*cos(deg2rad(90)), ...
%            'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
% 
% % 차량의 초기 위치와 자세 표시 (화살표)
% quiver_vehicle_x = quiver(state_history(1,1), state_history(2,1), ...
%            arrow_length*cos(state_history(5,1)), arrow_length*sin(state_history(5,1)), ...
%            'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
% quiver_vehicle_y = quiver(state_history(1,1), state_history(2,1), ...
%            -arrow_length*sin(state_history(5,1)), arrow_length*cos(state_history(5,1)), ...
%            'c', 'LineWidth', 2, 'MaxHeadSize', 0.5);
% 
% % 궤적을 나타내는 라인
% trajectory_handle = plot(state_history(1,1), state_history(2,1), 'b-', 'LineWidth', 1);
% 
% % 애니메이션 루프
% for i = 1:sim_step
%     % 차량의 현재 위치와 자세 업데이트
%     set(quiver_vehicle_x, 'XData', state_history(1,i), 'YData', state_history(2,i), ...
%                         'UData', arrow_length*cos(state_history(5,i)), ...
%                         'VData', arrow_length*sin(state_history(5,i)));
%     set(quiver_vehicle_y, 'XData', state_history(1,i), 'YData', state_history(2,i), ...
%                         'UData', -arrow_length*sin(state_history(5,i)), ...
%                         'VData', arrow_length*cos(state_history(5,i)));
% 
%     % 궤적 업데이트
%     set(trajectory_handle, 'XData', state_history(1,1:i), 'YData', state_history(2,1:i));
% 
%     % 현재 위치 점 업데이트
%     if i == 1
%         current_pos = scatter(state_history(1,i), state_history(2,i), 50, 'b', 'filled');
%     else
%         set(current_pos, 'XData', state_history(1,i), 'YData', state_history(2,i));
%     end
% 
%     drawnow;
% 
%     % 애니메이션 속도 조절 (필요에 따라 조정)
%     % pause(0.01);
% end
% 
% % 레전드 추가
% legend('Target Satellite', 'Pre-docking Point', 'Start Position', ...
%        'Target x-axis', 'Target y-axis', 'Vehicle x-axis', 'Vehicle y-axis', ...
%        'Trajectory', 'Current Position', ...
%        'Location', 'best');
