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
addpath(genpath('C:\Users\USER\Desktop\SAT_new2'));

%% Initialize setting
params = Params_init(); % Load Parameter struct

X = [ 517; 484; 0; 0; 0; 0 ]; % ( posX, posY, V_X, V_Y, rotZ, rateZ )
X_ss = X;
z = X; % ( posX, posY, V_X, V_Y, rotZ, rateZ )
U_binary = zeros( 8,1 ); % ( T1 T2 T3 T4 T5 T6 T7 T8 RW )

Rk = params.EKF.R;

Q_Fx = 0;
Q_Fy = 0;
Q_Tau = 0;
Qw = diag( [ Q_Fx.^2, Q_Fx.^2, Q_Tau.^2 ]' );

% Simulation setting
Main_Hz = params.System.MainLoopHz;
Sensor_VI_Hz = params.Sensor.VICON_HZ;
PID_control_Hz = Main_Hz;

sim_dt = 1 / Main_Hz; % [sec]
sim_Time = 60; % [sec]
sim_step = ceil( sim_Time / sim_dt );

% Sensor & Control interval Setting
Sensor_VI_interval = Main_Hz / Main_Hz; % sensor sampling set up
PID_control_interval = Main_Hz / PID_control_Hz; 
 
% Time duty initialize
real_time = 0;

% Data log initialize
state_history = zeros(6, sim_step);
temp3sigma_EKF = zeros(6, sim_step);
X_raw_hist = zeros(6, sim_step);
X_predict_hist = zeros(6, sim_step);
debug_control = zeros(9, sim_step);
t = zeros(1, sim_step);
target_history = zeros(6, sim_step);
debug_cmd_delta_hist = zeros(9, sim_step);
X_measure = zeros(6, sim_step);
X_filt_hist = zeros(6, sim_step);

% Main simulation loop
for i = 1 : sim_step
    
    X_target = generate_commands(real_time); % Generate - X_target & tolerance

    if mod(i-1, PID_control_interval) == 0 

        U = Controller_PID_Argument( X, X_target, sim_dt, params );

    end

    [ final_cmd, debug_cmd_delta ]  = Control_Allocator( U, params );

    % Stochastic RK4 simulation
    X = srk4( @vehicle_dynamics_Airbearing_stochastic, X, final_cmd, Qw, sim_dt, params, sim_dt );

    % data save
    state_history(:, i) = X;
    debug_control(:, i) = final_cmd;
    t(i) = real_time;
    target_history(:, i) = X_target;
    % debug_cmd_delta_hist(:, i) = debug_cmd_delta;

    real_time = real_time + sim_dt;
end

for superLoop = 1:1

X_filt = state_history( :, 1 );
P_ekf =  params.EKF.P;

for k = 1 : sim_step

    if mod(k-1, Sensor_VI_interval) == 0 

        X_ture = state_history( :, k );
        U_ture = debug_control( :, k );
        X_target_ekf = target_history( :, k );
        
        % z = [ X_ture(1); X_ture(2); X_ture(5) ] + randn( 3, 1 ) .* [ Rk(1,1), Rk(2,2), Rk(5,5) ]';
        z = X_ture + randn( 6, 1 ) .* [ Rk(1,1), Rk(2,2), Rk(3,3), Rk(4,4), Rk(5,5), Rk(6,6) ]';
        [ X_filt, P_ekf, y_hat, X_predict ] = processDataSIM_VI( z, P_ekf, X_filt, X_target_ekf, U_ture, params, 1/Sensor_VI_Hz );

    end

    X_filt_hist( :, k ) = X_filt; 
    temp3sigma_EKF( :, k ) = 3 * sqrt( diag( P_ekf ) );
    X_predict_hist( :, k ) = X_predict;
    % X_raw_hist( :, k ) = y_hat;

end

%% Plot the results

figure(1);
subplot(3, 1, 1);
plot(t, state_history(1, :), 'b', t, target_history(1, :), 'r--' );
hold on
% plot(t, X_filt_hist(1, :), 'g--' ,t, X_predict_hist(1, :), 'm--')
plot(t, X_filt_hist(1, :), 'g--' )

xlabel('[sec]')
ylabel('[mm]')
title('X Position');
legend('RK4', 'Target', 'filter');

subplot(3, 1, 2);
plot(t, state_history(2, :), 'b', t, target_history(2, :), 'r--' );
hold on
% plot( t, X_filt_hist(2, :), 'g--',t, X_predict_hist(2, :), 'm--' )
plot(t, X_filt_hist(2, :), 'g--' )

xlabel('[sec]')
ylabel('[mm]')
title('Y Position');
legend('RK4', 'Target', 'filter');

subplot(3, 1, 3);
plot(t, rad2deg(state_history(5, :)), 'b', t, rad2deg(target_history(5, :)));
hold on
% plot(t, rad2deg(X_filt_hist(5, :)), 'g--', t, rad2deg(X_predict_hist(5, :)), 'm--' )
plot(t, rad2deg(X_filt_hist(5, :)), 'g--')

xlabel('[sec]')
ylabel('[deg]')
title('Yaw Angle (psi)');
legend('RK4', 'Target', 'filter');

% figure(2);
% imagesc(t, 1:8, debug_control(1:8, :));
% colormap([92, 136, 196; 255, 253, 181] / 255);
% xlabel('Time [sec]');
% ylabel('Relay Number');
% title('Relay States Heatmap');
% yticks(1:8);
% grid on;
% c = colorbar;
% c.Ticks = [0.25 0.75];
% c.TickLabels = {'OFF', 'ON'};

% figure(3);
% for i = 1:8
% 
%     plot( t, debug_cmd_delta_hist(i, :) );  % 추력 (벡터 크기)
%     ylabel('Thrust (mN)');
%     grid on
%     hold on  
% 
% end
% legend('T1', 'T2', 'T3', 'T4', 'T5', 'T6','T7', 'T8' );

figure(4);

caxis = 1;

subplot(3,1,caxis)
plot( t, temp3sigma_EKF( caxis, : ),'r-');
title('Position Error, Estimate')
hold on;grid on ;
plot( t, state_history( caxis, : ) - X_filt_hist( caxis, : ),'b');
hold on;grid on ;
plot( t, -temp3sigma_EKF( caxis, : ),"r-");
grid on ;
hold on;
xlabel('Time(sec)');ylabel('X error [mm]');
ylim([-max(temp3sigma_EKF( caxis, : ))*2 max(temp3sigma_EKF( caxis, : ))*2 ]);

caxis = 2;

subplot(3,1,caxis)
plot( t ,temp3sigma_EKF( caxis, : ),'r-');
title('Position Error, Estimate')
hold on;grid on ;
plot( t , state_history( caxis, : ) - X_filt_hist( caxis, : ),'b');
hold on;grid on ;
plot( t, -temp3sigma_EKF( caxis, : ),"r-");
grid on ;
hold on;
xlabel('Time(sec)');ylabel('Y error [mm]');
ylim([-max(temp3sigma_EKF( caxis, : ))*2 max(temp3sigma_EKF( caxis, : ))*2 ]);

caxis = 5;

subplot(3,1,3)
plot( t, rad2deg( temp3sigma_EKF( caxis, : ) ),'r-');
title('Attitude Error , Estimate')
hold on;grid on ;
plot( t, rad2deg(state_history( caxis, : ) - X_filt_hist( caxis, : )),'b');
hold on;grid on ;
plot( t, -rad2deg( temp3sigma_EKF( caxis, : ) ),"r-");
grid on ;
hold on;
xlabel('Time(sec)');ylabel('psi error [deg]');
ylim([-max(rad2deg( temp3sigma_EKF( caxis, : )))*2 max(rad2deg( temp3sigma_EKF( caxis, : )) )*2 ]);

end