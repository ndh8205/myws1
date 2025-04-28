% Main Script
% This Code is for HANul Rocket Simulation
% 1st Project - Cold gas reaction control system
% Last update: 2024-11-25
% Made by NDH

% Initialize Matlab
clc;
clear all;
close all;

% Add Library directory (develope version path code)
addpath(genpath('C:\Users\USER\Desktop\hanul_GNC'));

% Auto library path (distribution version path code) 
%curdir = fileparts(mfilename('fullpath'));
%addpath(genpath(fullfile(curdir, 'hanul_GNC')));

% Simulation Parameters
params = Params_init_HANul(); % Load all simulation parameters

% Initialize State Vector
pos = [ 0, 0, 0 ]'; % Position XYZ [m] - Inertial frame  
V_B = [ 0, 0, 0 ]'; % Velocity uvw [m/s] - body frame 
att_euler = [ deg2rad(0), deg2rad(90), deg2rad(0) ]; % Attitude angle(Euler: roll, pitch, yaw) [rad] - Inertial frame
att_quat = GetQUAT( att_euler(3), att_euler(2), att_euler(1) )'; % Attitude Quaternion - Inertial frame 
omega = [ 0, 0, 0 ]'; % Angular rates [rad/s^2] - body frame 

X_int_q = [ pos; V_B; att_quat; omega ]; % Initial State Vector (Always initial value vector)
X = X_int_q; % Current State Vector initialize 

RI2B = GetDCM_Euler( att_euler(3), att_euler(2), att_euler(1) );
RB2I = RI2B';

% Simulation Setting

Sim_Loop_Hz = 400;
Sim_time = 30;

attitude_Control_Hz = 100;
Qf = diag( zeros(6,1) );

dt_sim = 1 / Sim_Loop_Hz;

t = (0 : Sim_time / dt_sim) * dt_sim;

num_steps = length(t);
state_history = zeros(length(X), num_steps);
state_history2 = zeros(13, num_steps); % Adjusted for X_Euler size
debug_q = zeros(4, num_steps);
angle_commands = zeros(3, num_steps);
rate_commands = zeros(3, num_steps);
Thrust_mass_history = zeros(4, num_steps);
Log_save = zeros(6, num_steps);

% Controller interval Setting
attitude_angle_interval = Sim_Loop_Hz / attitude_Control_Hz; 

% Initialize simulation time
real_time = 0;

% Initialize Waitbar
hWait = waitbar(0, '시뮬레이션 진행 중...', 'Name', '진행 상태');
% Set waitbar update interval to reduce overhead
update_interval = 100; % Update every 1000 steps

try
    % Main simulation loop
    for i = 1:num_steps

        [ Command_Vector ] = generate_commands_HANul(i, Sim_Loop_Hz);

        if mod(i-1, attitude_angle_interval) == 0

            U  = Controller_PPID_Argument( X, Command_Vector, dt_sim, params );

        end 

        [ final_cmd, debug_cmd ]  = Control_Allocator_RCS( U, params );

        [ X, Thrust_mass ] = srk4( @vehicle_dynamics_HANul, X, final_cmd, Qf, dt_sim, params, i, dt_sim );

        quat_now = X( 7:10 );
        Euler = Quat2Euler( quat_now' );
        X_Euler = X;
        X_Euler( 7:9 ) = rad2deg(Euler);
        X_Euler( 10:12 ) = X( 11:13 );

        % Data logging
        state_history(:, i) = X;
        state_history2(:, i) = X_Euler;
        Log_save(:, i) = [X_Euler(7:9); X(1:3)];
        debug_q(:, i) = quat_now;
        Thrust_mass_history(:, i) = Thrust_mass;

        real_time = real_time + dt_sim;

        % Update waitbar periodically
        if mod(i, update_interval) == 0 || i == num_steps
            progress = i / num_steps;
            waitbar(progress, hWait, sprintf('진행 중: %.2f%%', progress * 100));
        end
    end

    % Close Waitbar after completion
    close(hWait);

catch ME
    % Close waitbar in case of error
    close(hWait);
    rethrow(ME);
end

% Calculate Apogee and Impact Time

% Find the maximum altitude (apogee)
apogee = max(-state_history(3, :));
apogee_index = find(-state_history(3, :) == apogee, 1);

if isempty(apogee_index)
    apogee_time = NaN;
else
    apogee_time = t(apogee_index);
end


% Post-processing and plotting

% Plot Position and Velocity
figure(1);
titles = {'X Position', 'Y Position', 'Altitude (Z)', 'u Velocity', 'v Velocity', 'w Velocity'};
ylabels = {'Position [m]', 'Position [m]', 'Altitude [m]', 'Velocity [m/s]', 'Velocity [m/s]', 'Velocity [m/s]'};

for k = 1:6
    subplot(2, 3, k);
    switch k
        case 1  % X Position
            plot(t, state_history(1, :), 'b');
        case 2  % Y Position
            plot(t, state_history(2, :), 'g');
        case 3  % Altitude (Z)
            plot(t, -state_history(3, :), 'r');  % Altitude is negative of Z position
        case 4  % u Velocity
            plot(t, state_history(4, :), 'b');
        case 5  % v Velocity
            plot(t, state_history(5, :), 'g');
        case 6  % w Velocity
            plot(t, state_history(6, :), 'r');
    end
    xlabel('Time [s]');
    ylabel(ylabels{k});
    title(titles{k});
    grid on;
    hold on;

    % Add Apogee and Impact lines
    y_limits = ylim;
    if ~isnan(apogee_time)
        line([apogee_time apogee_time], y_limits, 'Color', 'r', 'LineStyle', '--', 'DisplayName', 'Apogee');
    end
    legend('show');
end

% Plot Attitude Angles and Angular Rates
figure(2);
titles = {'Roll Angle (\phi)', 'Pitch Angle (\theta)', 'Yaw Angle (\psi)', 'Angular Rate p', 'Angular Rate q', 'Angular Rate r'};
ylabels = {'Angle [deg]', 'Angle [deg]', 'Angle [deg]', 'Angular Rate [rad/s]', 'Angular Rate [rad/s]', 'Angular Rate [rad/s]'};

for k = 1:6
    subplot(2, 3, k);
    plot(t, state_history2( k+6 , : ));
    xlabel('Time [s]');
    ylabel(ylabels{k});
    title(titles{k});
    grid on;
    hold on;

    % Add Apogee and Impact lines
    y_limits = ylim;
    if ~isnan(apogee_time)
        line([apogee_time apogee_time], y_limits, 'Color', 'r', 'LineStyle', '--', 'DisplayName', 'Apogee');
    end
    legend('show');
end


% Plot Mass and Thrust over Time
figure(4);
yyaxis left;
plot(t, Thrust_mass_history(4, :), 'b');
ylabel('Mass [kg]');
ylim([min(Thrust_mass_history(4, :)) - 0.5, max(Thrust_mass_history(4, :)) + 0.5]);
yyaxis right;
thrust_magnitude = vecnorm(Thrust_mass_history(1:3, :));
plot(t, thrust_magnitude, 'r');
ylabel('Thrust [N]');
xlabel('Time [s]');
title('Rocket Mass and Thrust over Time');
legend('Mass', 'Thrust');
grid on;

