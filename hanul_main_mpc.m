%% Initialize Matlab
clc;
clear all;
close all;

% Add Library directory
addpath(genpath('C:\Users\USER\Desktop\hanul_GNC_ver1'));

% ===== Vehicle Geometry & Mass Properties load =====
xmlFilePath = 'rocket.xml';
rocketParams = extractRocketParams(xmlFilePath);
flatRocketParams = flattenRocketStructure(rocketParams);

% ===== Motor data Load =====
motorCsvPath = 'AeroTech_M2400T.csv';

% ===== Aerodynamics coeficent data Load =====
force_moment_path = 'csv_Force_moment_axiseq.csv';

% Simulation Parameters
params = vehicle_params(flatRocketParams, 0, motorCsvPath, true); % Initialize (reset=true)

% Initialize State Vector
pos = [ 0, 0, 0 ]'; % Position XYZ [m] - Inertial frame  
V_B = [ 0, 0, 0 ]'; % Velocity uvw [m/s] - body frame 
att_euler = [ deg2rad(0), deg2rad(88), deg2rad(0) ]; % Attitude angle(Euler: roll, pitch, yaw) [rad] - Inertial frame
att_quat = GetQUAT( att_euler(3), att_euler(2), att_euler(1) )'; % Attitude Quaternion - Inertial frame 
omega = [ 0, 0, 0 ]'; % Angular rates [rad/s^2] - body frame 

X_int_q = [ pos; V_B; att_quat; omega ]; % Initial State Vector
X = X_int_q; % Current State Vector initialize 

RI2B = GetDCM_Euler( att_euler(3), att_euler(2), att_euler(1) );
RB2I = RI2B';

% Simulation Setting
Sim_Loop_Hz = 400;
Sim_time = 30;

attitude_Control_Hz = 40;
Qf = diag( zeros(6,1) );

dt_sim = 1 / Sim_Loop_Hz;
t = (0 : Sim_time / dt_sim) * dt_sim;
num_steps = length(t);

% Control mode selection
control_mode = 2;  % 1: PPID, 2: Quaternion MPC
controller_names = {'PPID', 'MPC'};
disp(['Selected control mode: ', controller_names{control_mode}]);

% Initialize data logging variables
state_history = zeros(length(X), num_steps);
state_history2 = zeros(13, num_steps); % Including Euler angles version
debug_q = zeros(4, num_steps);
control_input_history = zeros(8, num_steps);
Thrust_mass_history = zeros(4, num_steps);
Log_save = zeros(6, num_steps);

% Additional variables for quaternion and command logging
target_quat_history = zeros(4, num_steps);
euler_command_history = zeros(3, num_steps);
quat_error_history = zeros(4, num_steps);
quat_error_magnitude = zeros(1, num_steps);

% MPC related variables
U_prev = zeros(8, 1); % Previous control input
mpc_computation_time = zeros(1, num_steps);
predicted_trajectories = cell(1, num_steps);

% Aerodynamic and moment data logging
alpha_tot_history = zeros(1, num_steps);
phi_A_history = zeros(1, num_steps);
F_aero_history = zeros(3, num_steps);
M_aero_history = zeros(3, num_steps);
moment_coupling_history = zeros(3, num_steps);
moment_coupling_magnitude = zeros(1, num_steps);
U_prev = zeros(8, 1); % Previous control input

% Controller interval Setting
attitude_angle_interval = Sim_Loop_Hz / attitude_Control_Hz; 

% Initialize simulation time
real_time = 0;

% Initialize Waitbar
hWait = waitbar(0, 'Simulation in progress...', 'Name', 'Progress');
update_interval = 100; % Update every 100 steps

try
    % Main simulation loop
    for i = 1:num_steps
        % Current simulation time
        current_time = (i-1) * dt_sim;
        
        if mod(i, 400) == 0
            disp(['Simulation time: ', num2str(current_time), ' seconds (', num2str(i/num_steps*100, '%.1f'), '% complete)']);
        end
        
        % Update parameters based on current time
        params = vehicle_params(flatRocketParams, current_time, motorCsvPath, false);
        
        % Generate commands (quaternion attitude command)
        Command_Vector1 = generate_commands_HANul(i, Sim_Loop_Hz, params);
        Command_Vector = Command_Vector1(2:5);
        
        % Convert quaternion command to Euler angles for storage
        euler_command_history(:, i) = rad2deg(Quat2Euler(Command_Vector'));
        target_quat_history(:, i) = Command_Vector;

        % Execute controller (at attitude_Control_Hz frequency)
        if mod(i-1, attitude_angle_interval) == 0
            switch control_mode
                case 1
                    % Execute PPID controller
                    tic;
                    U = Controller_PPID_Argument(X, Command_Vector, dt_sim, params);
                    mpc_computation_time(i) = toc;
                    [ final_cmd, debug_cmd ] = Control_Allocator_RCS(U, params);
                    
                case 2
                    % Execute quaternion-based MPC controller
                    try
                        % Debugging: Output state before controller call
                        if mod(i, 100) == 0
                            disp(['==== Step ', num2str(i), ' (', num2str(current_time), 's) ====']);
                            disp(['Quaternion: ', num2str(X(7:10)')]);
                            disp(['Angular velocity: ', num2str(X(11:13)')]);
                            disp(['Previous control input: ', num2str(U_prev')]);
                        end
                        
                        [U, comp_time, pred_traj] = Controller_MPC_HANul_Quaternion(X, Command_Vector, U_prev, 1/attitude_Control_Hz, params);
                        U_prev = U;
                        final_cmd = U; 
                        
                        % Debugging: Output control input
                        if mod(i, 100) == 0
                            disp(['Control input: ', num2str(final_cmd')]);
                            disp(['Computation time: ', num2str(comp_time*1000), ' ms']);
                            disp('==============================');
                        end
                        
                        % Save MPC debug information
                        mpc_computation_time(i) = comp_time;
                        predicted_trajectories{i} = pred_traj;
                    catch ME
                        warning('MPC controller error: %s', ME.message);
                        disp(['Error location: ', ME.stack(1).name, ' (line ', num2str(ME.stack(1).line), ')']);
                        
                        % Fallback to PPID on error
                        U = Controller_PPID_Argument(X, Command_Vector, dt_sim, params);
                        [ final_cmd, debug_cmd ] = Control_Allocator_RCS(U, params);
                        
                        % Maintain previous control input (for next attempt)
                        if exist('U', 'var')
                            U_prev = U;
                        end
                    end
            end
        end

        % Integrate dynamics
        [ X, Thrust_mass, alpha_tot, phi_A, F_aero, M_aero, moment_coupling ] = srk4(@vehicle_dynamics_HANul, X, final_cmd, Qf, dt_sim, params, current_time, dt_sim);

        % Attitude transformation (quaternion -> Euler)
        quat_now = X(7:10);
        Euler = Quat2Euler(quat_now'); % Convert to row vector
        X_Euler = X;
        X_Euler(7:9) = rad2deg(Euler);
        X_Euler(10:12) = X(11:13);
        
        % Calculate quaternion error
        quat_error = q2q_mult(quat_now, inv_q(Command_Vector'));
        quat_error_history(:, i) = quat_error;
        quat_error_magnitude(i) = 1 - abs(quat_error(1));

        % Record data
        state_history(:, i) = X;
        state_history2(:, i) = X_Euler;
        Log_save(:, i) = [X_Euler(7:9); X(1:3)];
        debug_q(:, i) = quat_now;
        control_input_history(:, i) = final_cmd;
        Thrust_mass_history(:, i) = Thrust_mass;

        alpha_tot_history(i) = alpha_tot;
        phi_A_history(i) = phi_A;
        F_aero_history(:,i) = F_aero;
        M_aero_history(:,i) = M_aero;
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

%% Result Analysis and Plots

% Calculate Apogee
apogee = max(-state_history(3, :));
apogee_index = find(-state_history(3, :) == apogee, 1);

if isempty(apogee_index)
    apogee_time = NaN;
else
    apogee_time = t(apogee_index);
end

% Figure 1: Position and Velocity
figure(1);
sgtitle(['HANul Rocket Simulation - ', controller_names{control_mode}, ' Controller']);

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
    
    % Add Apogee line
    if ~isnan(apogee_time) && k == 3
        hold on;
        line([apogee_time apogee_time], ylim, 'Color', 'k', 'LineStyle', '--', 'DisplayName', 'Apogee');
        legend('show');
    end
end

% Figure 2: Attitude and Angular Velocity - Including actual and command values
figure(2);
sgtitle(['HANul Control - ', controller_names{control_mode}, ' Controller']);

titles = {'Roll Angle (\phi)', 'Pitch Angle (\theta)', 'Yaw Angle (\psi)', 'Roll Rate (p)', 'Pitch Rate (q)', 'Yaw Rate (r)'};
ylabels = {'Angle [deg]', 'Angle [deg]', 'Angle [deg]', 'Angular Rate [rad/s]', 'Angular Rate [rad/s]', 'Angular Rate [rad/s]'};

% Target attitude angles (roll=0, pitch=88, yaw=0 or modify if using different command values)
target_angles = [0, 88, 0]; % Modify with desired target angles

for k = 1:6
    subplot(2, 3, k);
    
    if k <= 3  % Attitude angles
        plot(t, state_history2(k+6, :), 'b-', 'LineWidth', 1.5);
        hold on;
        plot(t, euler_command_history(k, :), 'r--', 'LineWidth', 1.5);
        
        % Display target value as text (on the right side of graph)
        target_val = euler_command_history(k, end);
        
        % Calculate step response metrics (if needed)
        if k == 2  % Pitch angle
            % 2% of the difference between initial value and target value
            settling_threshold = abs(state_history2(k+6, 1) - target_val) * 0.02;
            if settling_threshold < 1  % Set minimum threshold
                settling_threshold = 1;
            end
            
            % Time to reach ±2% of target value
            settling_time_idx = find(abs(state_history2(k+6, :) - target_val) <= settling_threshold, 1, 'first');
            if ~isempty(settling_time_idx)
                settling_time = t(settling_time_idx);
                line([settling_time settling_time], ylim, 'Color', 'g', 'LineStyle', ':', 'DisplayName', ['Settling time: ', num2str(settling_time, '%.2f'), 's']);
            end
        end
        
        % Calculate and display maximum overshoot (if needed)
        if k == 2  % Pitch angle
            error_sign = sign(target_val - state_history2(k+6, 1));
            if error_sign ~= 0
                % Overshoot is the maximum value beyond the target
                overshoot_candidates = error_sign * (state_history2(k+6, :) - target_val);
                [max_overshoot, overshoot_idx] = max(overshoot_candidates);
                
                if max_overshoot > 0  % If there is overshoot
                    overshot_val = state_history2(k+6, overshoot_idx);
                    % Mark overshoot point
                    plot(t(overshoot_idx), overshot_val, 'ro', 'MarkerSize', 6);
                    % Display overshoot value
                    overshoot_percent = max_overshoot / abs(target_val - state_history2(k+6, 1)) * 100;
                end
            end
        end
        
        legend({'Actual Angle', 'Target Angle'}, 'Location', 'best');
    else  % Angular velocity
        plot(t, state_history2(k+6, :), 'b-');
        % Add angular velocity target if available (typically 0)
        hold on;
        plot(t, zeros(size(t)), 'r--');
        legend({'Actual Angular Velocity', 'Target Angular Velocity'}, 'Location', 'best');
    end
    
    xlabel('Time [s]');
    ylabel(ylabels{k});
    title(titles{k});
    grid on;
end

% Figure 3: Control Input (replaced by Figure 9)

% Figure 4: Aerodynamic Forces and Moments
figure(4);
sgtitle(['Aerodynamic Force and Moment Analysis - ', controller_names{control_mode}, ' Controller']);

% Aerodynamic angles
subplot(2, 2, 1);
yyaxis left;
plot(t, rad2deg(alpha_tot_history));
ylabel('Total Angle of Attack [\alpha_{tot}, deg]');
yyaxis right;
plot(t, rad2deg(phi_A_history), 'r');
ylabel('Aerodynamic Roll Angle [\phi_A, deg]');
title('Aerodynamic Angles');
xlabel('Time [s]');
grid on;

% Aerodynamic force and moment
subplot(2, 2, 2);
yyaxis left;
plot(t, vecnorm(F_aero_history));
ylabel('Aerodynamic Force [N]');
yyaxis right;
plot(t, vecnorm(M_aero_history), 'r');
ylabel('Aerodynamic Moment [Nm]');
title('Aerodynamic Force and Moment Magnitude');
xlabel('Time [s]');
grid on;

% Coupling moment
subplot(2, 2, 3);
plot(t, moment_coupling_history);
title('Coupling Moment Components');
xlabel('Time [s]');
ylabel('Moment [Nm]');
legend('X', 'Y', 'Z');
grid on;

% Coupling moment magnitude
subplot(2, 2, 4);
plot(t, moment_coupling_magnitude);
title('Coupling Moment Magnitude');
xlabel('Time [s]');
ylabel('Magnitude [Nm]');
grid on;

% Figure 5: Coordinate System Diagnostics (PID code's Figure 5)
figure(5);
sgtitle('Coordinate System Diagnostics');

% Aerodynamic coefficients visualization
subplot(2,2,1);
plot(t, [Thrust_mass_history(1,:); Thrust_mass_history(2,:); Thrust_mass_history(3,:)]);
title('Thrust Vector (Body Frame)');
xlabel('Time [s]');
ylabel('Thrust [N]');
legend('X', 'Y', 'Z');
grid on;

% MRP position change visualization
subplot(2,2,2);
% MRP_history variable data usage
plot(t, [params.vehicle.MRP_x * ones(1, length(t)); 
         params.vehicle.MRP_y * ones(1, length(t)); 
         params.vehicle.MRP_z * ones(1, length(t))]);
title('MRP Position (Body Frame Reference)');
xlabel('Time [s]');
ylabel('Position [m]');
legend('X', 'Y', 'Z');
grid on;

% αtot and φA visualization
subplot(2,2,3);
yyaxis left;
plot(t, rad2deg(alpha_tot_history));
ylabel('αtot [deg]');
yyaxis right;
plot(t, rad2deg(phi_A_history), 'r');
ylabel('φA [deg]');
title('Aerodynamic Angles');
xlabel('Time [s]');
grid on;

% Aerodynamic force and moment visualization
subplot(2,2,4);
yyaxis left;
plot(t, vecnorm(F_aero_history));
ylabel('Aerodynamic Force [N]');
yyaxis right;
plot(t, vecnorm(M_aero_history), 'r');
ylabel('Aerodynamic Moment [Nm]');
title('Aerodynamic Force and Moment Magnitude');
xlabel('Time [s]');
grid on;

% Figure 6: moment_coupling visualization (PID code's Figure 6)
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

% Figure 7: Quaternion Control Analysis
figure(7);
sgtitle('Quaternion-Based Attitude Control Analysis');

% Plot actual vs target quaternion components - emphasize target values
subplot(2,2,1);
plot(t, debug_q(1,:), 'b-', 'LineWidth', 1.5); 
hold on;
plot(t, target_quat_history(1,:), 'r--', 'LineWidth', 1.5);
plot(t, debug_q(2,:), 'g-', 'LineWidth', 1.5); 
plot(t, target_quat_history(2,:), 'g--', 'LineWidth', 1.5);
plot(t, debug_q(3,:), 'm-', 'LineWidth', 1.5); 
plot(t, target_quat_history(3,:), 'm--', 'LineWidth', 1.5);
plot(t, debug_q(4,:), 'c-', 'LineWidth', 1.5); 
plot(t, target_quat_history(4,:), 'c--', 'LineWidth', 1.5);

% Display final target values as text
for i = 1:4
    target_val = target_quat_history(i, end);
    text(t(end), target_val, ['q', num2str(i-1), '=', num2str(target_val, '%.2f')], 'Color', 'r', 'FontWeight', 'bold');
end

title('Quaternion Components: Actual vs Target');
xlabel('Time [s]');
ylabel('Quaternion Value');
legend({'q0 Actual', 'q0 Target', 'q1 Actual', 'q1 Target', 'q2 Actual', 'q2 Target', 'q3 Actual', 'q3 Target'}, 'Location', 'best');
grid on;

% Quaternion error components
subplot(2,2,2);
plot(t, quat_error_history, 'LineWidth', 1.5);
title('Quaternion Error Components');
xlabel('Time [s]');
ylabel('Error Value');
legend({'e0', 'e1', 'e2', 'e3'});
grid on;

% Quaternion error magnitude (control performance indicator)
subplot(2,2,3);
plot(t, quat_error_magnitude, 'LineWidth', 1.5);
hold on;

% Calculate and display settling time
if ~isempty(quat_error_magnitude)
    threshold = 0.01;
    settling_idx = find(quat_error_magnitude < threshold, 1, 'first');
    
    if ~isempty(settling_idx)
        settling_time = t(settling_idx);
        % Display settling time
        line([settling_time settling_time], ylim, 'Color', 'g', 'LineStyle', ':', 'LineWidth', 1.5);
        
        % Display settling criterion line
        line([0 t(end)], [threshold threshold], 'Color', 'r', 'LineStyle', '-.', 'LineWidth', 1);
    end
end

title('Quaternion Error Magnitude');
xlabel('Time [s]');
ylabel('Error Magnitude');
grid on;
ylim([0, max(max(quat_error_magnitude), 0.1)]);  % Set reasonable y-axis range

% Figure 8: Euler Angle Error Analysis
figure(8);
sgtitle(['Euler Angle Error Analysis - ', controller_names{control_mode}, ' Controller']);

% Calculate Euler angle errors
euler_error = state_history2(7:9, :) - euler_command_history;

% Plot Euler angle errors
subplot(2, 1, 1);
plot(t, euler_error);
xlabel('Time [s]');
ylabel('Error [deg]');
title('Euler Angle Errors');
legend('Roll Error', 'Pitch Error', 'Yaw Error');
grid on;

% Plot error magnitude
subplot(2, 1, 2);
euler_error_magnitude = sqrt(sum(euler_error.^2, 1));
plot(t, euler_error_magnitude);
xlabel('Time [s]');
ylabel('Error Magnitude [deg]');
title('Total Euler Angle Error Magnitude');
grid on;

% Figure 9: Control Input
figure(9);
sgtitle(['Control Input - ', controller_names{control_mode}, ' Controller']);

% Canard control input
subplot(2, 1, 1);
plot(t, rad2deg(control_input_history(1:4, :)), 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Canard Angle [deg]');
title('Canard Control');

% Display maximum allowed angle
max_canard_angle = max(max(rad2deg(control_input_history(1:4, :))));
min_canard_angle = min(min(rad2deg(control_input_history(1:4, :))));
canard_limit = max(abs(max_canard_angle), abs(min_canard_angle));

hold on;
line([0 t(end)], [canard_limit canard_limit], 'Color', 'r', 'LineStyle', '-.', 'LineWidth', 1);
line([0 t(end)], [-canard_limit -canard_limit], 'Color', 'r', 'LineStyle', '-.', 'LineWidth', 1);
text(t(end), canard_limit, ['Limit: ±', num2str(canard_limit, '%.1f'), '°'], 'Color', 'r', 'FontWeight', 'bold');

legend({'Canard 1', 'Canard 2', 'Canard 3', 'Canard 4'});
grid on;

% RCS thruster control input
subplot(2, 1, 2);
stairs(t, control_input_history(5:8, :)', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('RCS State (ON/OFF)');
title('RCS Thruster Control');
legend({'RCS 1', 'RCS 2', 'RCS 3', 'RCS 4'});
ylim([-0.1, 1.1]);
grid on;

% Figure 10 (newly added): MPC-specific plot - Control target time and predicted trajectories
figure(10);
sgtitle(['MPC Performance Analysis - ', controller_names{control_mode}, ' Controller']);

% MPC target time vs actual computation time comparison
target_time = (1/attitude_Control_Hz) * 1000; % Target control period (ms)
plot(t, mpc_computation_time * 1000, 'b-', 'LineWidth', 1.5); % Actual computation time (ms)
hold on;
plot(t, target_time * ones(size(t)), 'r--', 'LineWidth', 1.5); % Target time

% Important: Clearly display the target computation time limit
text(t(end)*0.05, target_time*1.1, ['Target computation time: ' num2str(target_time, '%.1f') ' ms'], 'Color', 'r', 'FontWeight', 'bold');
ylabel('Computation Time [ms]');
xlabel('Simulation Time [s]');
title(['MPC Controller Computation Time Limit (' num2str(target_time, '%.1f') ' ms) vs Actual Computation Time']);
legend('Actual Computation Time', 'Time Limit');
grid on;

% Highlight parts where computation time exceeds target
exceed_indices = find(mpc_computation_time * 1000 > target_time);
if ~isempty(exceed_indices)
    plot(t(exceed_indices), mpc_computation_time(exceed_indices) * 1000, 'ro', 'MarkerSize', 4);
    legend('Actual Computation Time', 'Time Limit', 'Time Limit Exceeded');
end

%% Calculate and Output Performance Metrics

% Calculate attitude error RMSE
euler_error = zeros(3, num_steps);
for i = 1:3
    euler_error(i,:) = state_history2(i+6,:) - euler_command_history(i,:);
end
rmse_attitude = sqrt(mean(euler_error.^2, 2));

% Calculate angular velocity RMSE
omega_rmse = sqrt(mean(state_history2(10:12,:).^2, 2));

% Control input usage
canard_usage = mean(abs(control_input_history(1:4,:)), 2);
rcs_usage = mean(control_input_history(5:8,:), 2);

% Calculate settling time
settling_time = zeros(3, 1);
for i = 1:3
    % Roll, Yaw: target is 0 degrees, Pitch: target is around 88 degrees
    target_val = euler_command_history(i, end);
    
    threshold = abs(target_val) * 0.02;  % ±2% of target
    if threshold < 2  % Set minimum threshold (2 degrees)
        threshold = 2;
    end
    
    idx = find(abs(state_history2(i+6,:) - target_val) <= threshold, 1, 'first');
    if ~isempty(idx)
        settling_time(i) = t(idx);
    else
        settling_time(i) = Inf;  % Not settled
    end
end

% Output results
fprintf('\n======= HANul Rocket Simulation Results =======\n');
fprintf('Controller Type: %s\n\n', controller_names{control_mode});

fprintf('1. Attitude Control Accuracy (RMSE):\n');
fprintf('   Roll: %.2f deg\n', rmse_attitude(1));
fprintf('   Pitch: %.2f deg\n', rmse_attitude(2));
fprintf('   Yaw: %.2f deg\n\n', rmse_attitude(3));

fprintf('2. Angular Velocity Stability (RMSE):\n');
fprintf('   Roll rate: %.4f rad/s\n', omega_rmse(1));
fprintf('   Pitch rate: %.4f rad/s\n', omega_rmse(2));
fprintf('   Yaw rate: %.4f rad/s\n\n', omega_rmse(3));

fprintf('3. Control Input Usage:\n');
fprintf('   Average canard usage: %.2f deg\n', rad2deg(mean(canard_usage)));
fprintf('   Average RCS thruster usage rate: %.2f%%\n\n', mean(rcs_usage)*100);

fprintf('4. Attitude Settling Time:\n');
if isinf(settling_time(1))
    fprintf('   Roll: Not settled\n');
else
    fprintf('   Roll: %.2f seconds\n', settling_time(1));
end

if isinf(settling_time(2))
    fprintf('   Pitch: Not settled\n');
else
    fprintf('   Pitch: %.2f seconds\n', settling_time(2));
end

if isinf(settling_time(3))
    fprintf('   Yaw: Not settled\n\n');
else
    fprintf('   Yaw: %.2f seconds\n\n', settling_time(3));
end

fprintf('5. MPC Computation Time Analysis:\n');
fprintf('   Average computation time: %.2f ms\n', mean(mpc_computation_time)*1000);
fprintf('   Maximum computation time: %.2f ms\n', max(mpc_computation_time)*1000);
fprintf('   Target control period: %.2f ms\n', target_time);
if ~isempty(exceed_indices)
    fprintf('   Time limit exceeded count: %d (%.1f%%)\n\n', length(exceed_indices), length(exceed_indices)/length(t)*100);
else
    fprintf('   No time limit exceeded\n\n');
end

fprintf('6. Flight Performance:\n');
fprintf('   Maximum altitude: %.2f m\n', apogee);
if ~isnan(apogee_time)
    fprintf('   Time to maximum altitude: %.2f seconds\n', apogee_time);
end

fprintf('=======================================\n');