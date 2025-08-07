%% Initialize Matlab
clc;
clear all;
close all;

% Add Library directory (develope version path code)
addpath(genpath('C:\Users\USER\Desktop\hanul_GNC_ver1'));

% ===== Vehicle Geometry & Mass Properties load =====
xmlFilePath = 'rocket.xml';
rocketParams = extractRocketParams(xmlFilePath);
flatRocketParams = flattenRocketStructure(rocketParams);

% ===== Motor data Load =====
motorCsvPath = 'AeroTech_M2400T.csv';

% ===== Aerodynamics coeficent data Load =====
force_moment_path = 'csv_Force_moment_axiseq.csv';  % Static aerodynamic coefficient file

% Simulation Parameters
% Use vehicle_params() function instead of old Params_init_HANul()
params = vehicle_params(flatRocketParams, 0, motorCsvPath, true); % Initialize (reset=true)

% Initialize State Vector
pos = [ 0, 0, 0 ]'; % Position XYZ [m] - Inertial frame  
V_B = [ 0, 0, 0 ]'; % Velocity uvw [m/s] - body frame 
att_euler = [ deg2rad(0), deg2rad(87), deg2rad(0) ]; % Attitude angle(Euler: roll, pitch, yaw) [rad] - Inertial frame
att_quat = GetQUAT( att_euler(3), att_euler(2), att_euler(1) )'; % Attitude Quaternion - Inertial frame 
omega = [ 0, 0, 0 ]'; % Angular rates [rad/s^2] - body frame 

X_int_q = [ pos; V_B; att_quat; omega ]; % Initial State Vector (Always initial value vector)
X = X_int_q; % Current State Vector initialize 

RI2B = GetDCM_Euler( att_euler(3), att_euler(2), att_euler(1) );
RB2I = RI2B';

U = zeros(6,1);

% Simulation Setting
Sim_Loop_Hz = 400;
Sim_time = 30;

attitude_Control_Hz = 40;
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

% Initialize history variables before simulation loop
alpha_tot_history = zeros(1, num_steps);
phi_A_history = zeros(1, num_steps);
F_aero_history = zeros(3, num_steps);
M_aero_history = zeros(3, num_steps);
MRP_history = zeros(3, num_steps);
control_input_history = zeros(8, num_steps);  % For 4 canards and 4 RCS thrusters

target_quat_history = zeros(4, num_steps);
euler_command_history = zeros(3, num_steps);
quat_error_history = zeros(4, num_steps);
quat_error_magnitude = zeros(1, num_steps);

moment_coupling_history = zeros(3, num_steps);
moment_coupling_magnitude = zeros(1, num_steps);

% Controller interval Setting
attitude_angle_interval = Sim_Loop_Hz / attitude_Control_Hz; 

% Initialize simulation time
real_time = 0;

% Initialize Waitbar
hWait = waitbar(0, 'Simulation in progress...', 'Name', 'Progress');
% Set waitbar update interval to reduce overhead
update_interval = 100; % Update every 100 steps

try
    % Main simulation loop
    for i = 1:num_steps
        % Current simulation time
        current_time = (i-1) * dt_sim;
        disp('now time')
        disp(current_time)
        
        % Update parameters based on current time (call vehicle_params function)
        params = vehicle_params(flatRocketParams, current_time, motorCsvPath, false);
        
        % Generate commands
        [ Command_Vector ] = generate_commands_HANul(i, Sim_Loop_Hz);

        % Execute attitude controller (at attitude_Control_Hz frequency)
        if mod(i-1, attitude_angle_interval) == 0
            U(4:6) = Controller_PPID_Argument(X, Command_Vector(2:5), dt_sim, params);
        end 
        
        U(1) = Command_Vector(1);

        % Allocate control commands
        [ final_cmd, debug_cmd ] = Control_Allocator_RCS(U, params);

        for li_nu = 2 : 5 

            final_cmd(li_nu) = LIMIT2( final_cmd(li_nu),-0.0873 ,0.0873 );

        end

        % Integrate dynamics - pass updated parameters
        [ X, Thrust_mass, alpha_tot, phi_A, F_aero, M_aero, moment_coupling ] = srk4(@vehicle_dynamics_HANul_pid, X, final_cmd, Qf, dt_sim, params, current_time, dt_sim);

        % Attitude transformation (quaternion -> Euler)
        quat_now = X(7:10);
        Euler = Quat2Euler(quat_now');
        X_Euler = X;
        X_Euler(7:9) = rad2deg(Euler);
        X_Euler(10:12) = X(11:13);

        quat_error = q2q_mult(quat_now, inv_q(Command_Vector(2:5)'));
        quat_error_history(:, i) = quat_error;
        quat_error_magnitude(i) = 1 - abs(quat_error(1));


        % Record data
        state_history(:, i) = X;
        state_history2(:, i) = X_Euler;
        Log_save(:, i) = [X_Euler(7:9); X(1:3)];
        debug_q(:, i) = quat_now;
        Thrust_mass_history(:, i) = Thrust_mass;

        % Store control inputs
        control_input_history(:, i) = [final_cmd(2:5); final_cmd(6:9)];

        alpha_tot_history(i) = alpha_tot;
        target_quat_history(:, i) = Command_Vector(2:5);
        euler_command_history(:, i) = rad2deg(Quat2Euler(Command_Vector(2:5)));

        phi_A_history(i) = phi_A;
        F_aero_history(:,i) = F_aero;
        M_aero_history(:,i) = M_aero;
        MRP_history(:,i) = [params.vehicle.MRP_x; params.vehicle.MRP_y; params.vehicle.MRP_z];

        moment_coupling_history(:,i) = moment_coupling;
        moment_coupling_magnitude(i) = norm(moment_coupling);

        real_time = real_time + dt_sim;

        % Update progress periodically
        if mod(i, update_interval) == 0 || i == num_steps
            progress = i / num_steps;
            waitbar(progress, hWait, sprintf('Progress: %.2f%%', progress * 100));
        end
    end

    % Close waitbar when complete
    close(hWait);

catch ME
    % Close waitbar on error
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

figure(2);
titles = {'Roll Angle (\phi)', 'Pitch Angle (\theta)', 'Yaw Angle (\psi)', 'Angular Rate p', 'Angular Rate q', 'Angular Rate r'};
ylabels = {'Angle [deg]', 'Angle [deg]', 'Angle [deg]', 'Angular Rate [rad/s]', 'Angular Rate [rad/s]', 'Angular Rate [rad/s]'};

for k = 1:6
    subplot(2, 3, k);
    if k <= 3
        % For attitude angles, plot both actual and command values
        plot(t, state_history2(k+6, :), 'b-', 'LineWidth', 1.5);
        hold on;
        plot(t, euler_command_history(k, :), 'r--', 'LineWidth', 1);
        legend('Actual', 'Command');
    else
        % For angular rates, plot only actual values
        plot(t, state_history2(k+6, :), 'b-');
    end
    xlabel('Time [s]');
    ylabel(ylabels{k});
    title(titles{k});
    grid on;
    
    % Add Apogee line
    y_limits = ylim;
    if ~isnan(apogee_time)
        line([apogee_time apogee_time], y_limits, 'Color', 'r', 'LineStyle', ':', 'DisplayName', 'Apogee');
    end
end


% Diagnostic graphs to be added after existing code
figure(5);
sgtitle('Coordinate System Diagnostics');

% Aerodynamic coefficient visualization
subplot(2,2,1);
plot(t, [Thrust_mass_history(1,:); Thrust_mass_history(2,:); Thrust_mass_history(3,:)]);
title('Thrust Vector (Body Frame)');
xlabel('Time [s]');
ylabel('Thrust [N]');
legend('X', 'Y', 'Z');
grid on;

% MRP position change visualization
subplot(2,2,2);
% Extract MRP position from params structure
% Assumes MRP_history is already stored
if exist('MRP_history', 'var')
    plot(t, MRP_history);
    title('MRP Position (Body Frame Reference)');
    xlabel('Time [s]');
    ylabel('Position [m]');
    legend('X', 'Y', 'Z');
    grid on;
else
    text(0.5, 0.5, 'MRP data not available', 'HorizontalAlignment', 'center');
end

% αtot and φA visualization
subplot(2,2,3);
% Assumes alpha_tot_history and phi_A_history are already stored
if exist('alpha_tot_history', 'var') && exist('phi_A_history', 'var')
    yyaxis left;
    plot(t, rad2deg(alpha_tot_history));
    ylabel('αtot [deg]');
    yyaxis right;
    plot(t, rad2deg(phi_A_history), 'r');
    ylabel('φA [deg]');
    title('Aerodynamic Angles');
    xlabel('Time [s]');
    grid on;
else
    text(0.5, 0.5, 'Angle data not available', 'HorizontalAlignment', 'center');
end

% Aerodynamic force and moment visualization
subplot(2,2,4);
% Assumes F_aero_history and M_aero_history are already stored
if exist('F_aero_history', 'var') && exist('M_aero_history', 'var')
    yyaxis left;
    plot(t, vecnorm(F_aero_history));
    ylabel('Aerodynamic Force [N]');
    yyaxis right;
    plot(t, vecnorm(M_aero_history), 'r');
    ylabel('Aerodynamic Moment [Nm]');
    title('Aerodynamic Force and Moment Magnitude');
    xlabel('Time [s]');
    grid on;
else
    text(0.5, 0.5, 'Aerodynamic data not available', 'HorizontalAlignment', 'center');
end

% moment_coupling visualization (add at the end of main code)
figure(6);
subplot(2,1,1);
plot(t, moment_coupling_history);
title('Coupling Moment (MRP_b × F_p)');
xlabel('Time [s]');
ylabel('Moment [Nm]');
legend('X', 'Y', 'Z');
grid on;

subplot(2,1,2);
plot(t, moment_coupling_magnitude);
title('Coupling Moment Magnitude');
xlabel('Time [s]');
ylabel('Magnitude [Nm]');
grid on;

figure(7);
sgtitle('Quaternion-Based Attitude Control Analysis');

% Plot actual vs target quaternion components
subplot(2,2,1);
plot(t, debug_q(1,:), 'b-', t, target_quat_history(1,:), 'b--');
hold on;
plot(t, debug_q(2,:), 'r-', t, target_quat_history(2,:), 'r--');
plot(t, debug_q(3,:), 'g-', t, target_quat_history(3,:), 'g--');
plot(t, debug_q(4,:), 'm-', t, target_quat_history(4,:), 'm--');
title('Quaternion Components: Actual vs Target');
xlabel('Time [s]');
ylabel('Quaternion Value');
legend('q0 Actual', 'q0 Target', 'q1 Actual', 'q1 Target', 'q2 Actual', 'q2 Target', 'q3 Actual', 'q3 Target');
grid on;

% Plot quaternion error components
subplot(2,2,2);
plot(t, quat_error_history);
title('Quaternion Error Components');
xlabel('Time [s]');
ylabel('Error Value');
legend('e0', 'e1', 'e2', 'e3');
grid on;

% Plot quaternion error magnitude (control performance metric)
subplot(2,2,3);
plot(t, quat_error_magnitude);
title('Quaternion Error Magnitude');
xlabel('Time [s]');
ylabel('Error Magnitude');
grid on;
ylim([0, max(max(quat_error_magnitude), 0.1)]);  % Set reasonable y-axis limits

figure(9);  % Using figure 9 since you already have figures 1-8
sgtitle(['Control Input - ', 'PID', ' Controller']);
% Canard control input
subplot(2, 1, 1);
plot(t, rad2deg(control_input_history(1:4, :)));
xlabel('Time [s]');
ylabel('Canard Angle [deg]');
title('Canard Control');
legend('Canard 1', 'Canard 2', 'Canard 3', 'Canard 4');
grid on;
% RCS thruster control input
subplot(2, 1, 2);
stairs(t, control_input_history(5:8, :)');
xlabel('Time [s]');
ylabel('RCS State (ON/OFF)');
title('RCS Thruster Control');
legend('RCS 1', 'RCS 2', 'RCS 3', 'RCS 4');
ylim([-0.1, 1.1]);
grid on;