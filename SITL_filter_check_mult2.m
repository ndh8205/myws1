% Main Script with IMM Integration
% This Code is SITL (Software in the loop)
% Ver.2.0.1 (IMM with EKF and UKF)
% 2024-08-12 / Controla Project
% Made by NDH & LJC

%% Initialize Matlab
clc;
clear all;
close all;

%% Add Library directory
addpath(genpath('C:\Users\USER\Desktop\Kalman_Filter\Sat_ver2\main'));

%% Initialize settings
params = Params_init(); % Load Parameter struct

% Initial States
X = [ 517; 484; 0; 0; 0; 0 ]; % ( posX, posY, V_X, V_Y, rotZ, rateZ )
Master_point = [ 2400; 2400; 0; 0; 0; 0 ]; % Initialize Mothership position and orientation

Rk = params.EKF.Rm;

Q_Fx = 0;
Q_Fy = 0;
Q_Tau = 0;
Qw = diag( [ Q_Fx.^2, Q_Fy.^2, Q_Tau.^2 ] );

% Simulation settings
Main_Hz = params.System.MainLoopHz;
Sensor_VI_Hz = params.Sensor.VICON_HZ;
PID_control_Hz = Main_Hz;
OPENCV_HZ = params.Sensor.OPENCV_HZ;

sim_dt = 1 / Main_Hz; % [sec]
sim_Time = 160; % [sec]
sim_step = ceil( sim_Time / sim_dt );

% Sensor & Control interval Setting
Sensor_interval = Main_Hz / OPENCV_HZ; % sensor sampling set up
PID_control_interval = Main_Hz / PID_control_Hz;

% Time duty initialize
real_time = 0;

% Data log initialize
state_history = zeros(6, sim_step);
Masterpoint_history = zeros(6, sim_step);
temp3sigma_EKF = zeros(6, sim_step);
X_raw_hist = zeros(6, sim_step);
X_predict_hist = zeros(6, sim_step);
debug_control = zeros(9, sim_step);
t = zeros(1, sim_step);
target_history = zeros(6, sim_step);
debug_cmd_delta_hist = zeros(9, sim_step);
X_measure = zeros(6, sim_step);
X_filt_hist = zeros(6, sim_step);
z_true_hist = zeros(3, sim_step);
state_filter_hist = zeros(3, sim_step);
z_measurement_hist = zeros(3, sim_step);

%% Initialize IMM Parameters

% Define model transition probabilities
% Rows: current model, Columns: next model
transition_prob = [0.75, 0.25;  % CV to CA
                   0.25, 0.75]; % CA to CV

num_models = 2;

% Initialize model probabilities
mu = [0.5; 0.5];  % Start with equal probability for EKF and UKF

% Initialize states and covariances for each model
X_models = cell(num_models, 1);
P_models = cell(num_models, 1);

% Initialize EKF
X_models{1} = X;             % EKF initial state
P_models{1} = params.EKF.P;  % EKF initial covariance

% Initialize UKF
X_models{2} = X;             % UKF initial state
P_models{2} = params.EKF.P;  % UKF initial covariance (조정 가능)

% Preallocate IMM history
X_IMM_hist = zeros(6, sim_step);
P_IMM_hist = zeros(6, 6, sim_step);
mu_hist = zeros(num_models, sim_step);

%% Main simulation loop with IMM

for i = 1 : sim_step
    
    % Generate target commands
    X_target = generate_commands(real_time); % Generate - X_target & tolerance
    
    % PID Control
    if mod(i-1, PID_control_interval) == 0 
        U = Controller_PID_Argument( X, X_target, sim_dt, params );
    end
    
    [ final_cmd, debug_cmd_delta ]  = Control_Allocator( U, params );
    
    % Stochastic RK4 simulation
    X = srk4( @vehicle_dynamics_Airbearing_stochastic, X, final_cmd, Qw, sim_dt, params, sim_dt );
    
    % Data save
    state_history(:, i) = X;
    Masterpoint_history(:, i) = Master_point;
    debug_control(:, i) = final_cmd;
    t(i) = real_time;
    target_history(:, i) = X_target;
    
    real_time = real_time + sim_dt;
    
    %% Sensor Simulation
    
    if mod(i-1, Sensor_interval) == 0 

        X_true = state_history( :, i );
        U_true = debug_control( :, i );
        X_target_ekf = target_history( :, i );
        master_point_current = Masterpoint_history( :, i );

        % Simulate sensor measurements
        [ z_AHRS, z_AHRS_true ] = measure_sensor_AHRS( X_true, master_point_current, Rk );
        [ z_Aruco, z_Aruco_true ] = measure_sensor_Aruco( X_true, master_point_current, Rk );

        z = [ z_Aruco; z_AHRS ];
        z_true = [ z_Aruco_true; z_AHRS_true ];

        % Save measurements
        z_measurement_hist(:,i) = z;
        z_true_hist(:,i) = z_true;

    else
        z = []; % No measurement at this step
    end
    
   %% IMM Steps

    % 1. Interaction (Mixing)
    c_j = transition_prob' * mu;  % Mixing probabilities
    
    % Initialize mixed state and covariance
    X_mix = cell(num_models,1);
    P_mix = cell(num_models,1);
    
    for j = 1:num_models
        X_mix{j} = zeros(6,1);
        for k = 1:num_models
            mix_prob = (transition_prob(k,j) * mu(k)) / c_j(j);
            X_mix{j} = X_mix{j} + mix_prob * X_models{k};
        end
        P_mix{j} = zeros(6,6);
        for k = 1:num_models
            mix_prob = (transition_prob(k,j) * mu(k)) / c_j(j);
            dx = X_models{k} - X_mix{j};
            P_mix{j} = P_mix{j} + mix_prob * (P_models{k} + dx * dx');
        end
    end
    
    % 2. Model Update (Run each filter)
    X_upd = cell(num_models,1);
    P_upd = cell(num_models,1);
    yhat_models = cell(num_models,1);
    S_models = cell(num_models,1);
    log_likelihood = zeros(num_models,1);
    
    for j = 1:num_models
        if j == 1
            % EKF Update
            if ~isempty(z)
                [X_upd{j}, P_upd{j}, yhat_models{j}, ~, S_models{j}] = processDataSIM_Muti_EKF_CV_imm(z, P_mix{j}, X_mix{j}, X_target, final_cmd, params, sim_dt);
            else
                X_upd{j} = X_mix{j};
                P_upd{j} = P_mix{j};
                yhat_models{j} = [];
                S_models{j} = [];
            end
        elseif j == 2
            % CA Model Update
            if ~isempty(z)
                [X_upd{j}, P_upd{j}, yhat_models{j}, ~, S_models{j}] = processDataSIM_Muti_EKF_CA_imm(z, P_mix{j}, X_mix{j}, X_target, final_cmd, params, sim_dt);
            else
                X_upd{j} = X_mix{j};
                P_upd{j} = P_mix{j};
                yhat_models{j} = [];
                S_models{j} = [];
            end
        end
    
        % Compute log-likelihood
        if ~isempty(z)
            if ~isempty(yhat_models{j}) && ~isempty(S_models{j})
                S_total = S_models{j};
                if min(eig(S_total)) <= 0
                    warning('S is not positive definite for model %d at step %d. Adjusting...', j, i);
                    S_total = S_total + eye(size(S_total)) * 1e-6;
                end
                log_likelihood(j) = log_mvnpdf(yhat_models{j}, zeros(size(yhat_models{j})), S_total);
            else
                log_likelihood(j) = -inf; % Assign negative infinity if computation fails
            end
        else
            log_likelihood(j) = 0; % Zero log-likelihood when no measurement is available
        end
    end
    
    % 3. Update Model Probabilities in log domain
    log_mu = log(c_j) + log_likelihood;
    max_log_mu = max(log_mu);
    log_mu = log_mu - max_log_mu; % For numerical stability
    mu = exp(log_mu);
    if sum(mu) == 0
        mu = ones(num_models, 1) / num_models; % Reset to uniform distribution
    else
        mu = mu / sum(mu); % Normalize
    end
    
    % 4. Combination (Blending)
    X_IMM = zeros(6,1);
    P_IMM = zeros(6,6);
    for j = 1:num_models
        X_IMM = X_IMM + mu(j) * X_upd{j};
    end
    for j = 1:num_models
        dx = X_upd{j} - X_IMM;
        P_IMM = P_IMM + mu(j) * (P_upd{j} + dx * dx');
    end
    
    % Save IMM estimates
    X_IMM_hist(:,i) = X_IMM;
    P_IMM_hist(:,:,i) = P_IMM;
    mu_hist(:,i) = mu;
    
    % Update models for next iteration
    for j = 1:num_models
        X_models{j} = X_upd{j};
        P_models{j} = P_upd{j};
    end

    %% Save filtered state
    if ~isempty(z)
        X_filt_hist(:,i) = X_IMM;
    else
        X_filt_hist(:,i) = X_mix{1}; % 필터 업데이트가 없을 경우 혼합 상태 사용
    end

end

%% Plot the results

% Plot Trajectories
figure(1);
subplot(3, 1, 1);
plot(t, state_history(1, :), 'b', t, target_history(1, :), 'r--' );
hold on
plot(t, X_IMM_hist(1, :), 'k-', 'LineWidth',1.5 )
xlabel('[sec]')
ylabel('[mm]')
title('X Position');
legend('RK4', 'Target', 'IMM');

subplot(3, 1, 2);
plot(t, state_history(2, :), 'b', t, target_history(2, :), 'r--' );
hold on
plot(t, X_IMM_hist(2, :), 'k-', 'LineWidth',1.5 )
xlabel('[sec]')
ylabel('[mm]')
title('Y Position');
legend('RK4', 'Target', 'IMM');

subplot(3, 1, 3);
plot(t, rad2deg(state_history(5, :)), 'b', t, rad2deg(target_history(5, :)), 'r--');
hold on
plot(t, rad2deg(X_IMM_hist(5, :)), 'k-', 'LineWidth',1.5 )
xlabel('[sec]')
ylabel('[deg]')
title('Yaw Angle (psi)');
legend('RK4', 'Target', 'IMM');

% Plot Errors with 3-sigma bounds
figure(2);

% X Position Error
subplot(3,1,1)
sigma_x = squeeze(sqrt(P_IMM_hist(1,1,:)))'; % sim_step x 1 벡터
plot(t, 3*sigma_x, 'r-', 'LineWidth',1, 'DisplayName', '3-Sigma Upper');
hold on; grid on;
plot(t, state_history(1, :) - X_IMM_hist(1, :), 'b', 'DisplayName', 'Error');
plot(t, -3*sigma_x, 'r-', 'LineWidth',1, 'DisplayName', '-3-Sigma Lower');
xlabel('Time(sec)');
ylabel('X error [mm]');
title('X Position Error with 3-sigma Bounds');
legend('show');

% Y Position Error
subplot(3,1,2)
sigma_y = squeeze(sqrt(P_IMM_hist(2,2,:)))'; % sim_step x 1 벡터
plot(t, 3*sigma_y, 'r-', 'LineWidth',1, 'DisplayName', '3-Sigma Upper');
hold on; grid on;
plot(t, state_history(2, :) - X_IMM_hist(2, :), 'b', 'DisplayName', 'Error');
plot(t, -3*sigma_y, 'r-', 'LineWidth',1, 'DisplayName', '-3-Sigma Lower');
xlabel('Time(sec)');
ylabel('Y error [mm]');
title('Y Position Error with 3-sigma Bounds');
legend('show');

% Yaw Angle Error
subplot(3,1,3)
sigma_psi = squeeze(sqrt(P_IMM_hist(5,5,:)))'; % sim_step x 1 벡터
plot(t, rad2deg(3*sigma_psi), 'r-', 'LineWidth',1, 'DisplayName', '3-Sigma Upper');
hold on; grid on;
plot(t, rad2deg(state_history(5, :) - X_IMM_hist(5, :)), 'b', 'DisplayName', 'Error');
plot(t, rad2deg(-3*sigma_psi), 'r-', 'LineWidth',1, 'DisplayName', '-3-Sigma Lower');
xlabel('Time(sec)');
ylabel('Yaw error [deg]');
title('Yaw Angle Error with 3-sigma Bounds');
legend('show');

% % Plot Measurements
% figure(3);
% subplot(3, 1, 1);
% plot(t, z_measurement_hist(1, :), 'r:', t, z_true_hist(1, :), 'k', t, state_filter_hist(1, :), 'g--' );
% xlabel('[sec]')
% ylabel('[mm]')
% title('rho');
% legend('rho measurement', 'rho true');
% 
% subplot(3, 1, 2);
% plot(t, rad2deg(z_measurement_hist(2, :)), 'r:', t, rad2deg(z_true_hist(2, :)), 'k' , t, rad2deg(state_filter_hist(2, :)), 'g--');
% xlabel('[sec]')
% ylabel('[deg]')
% title('theta');
% legend('theta measurement', 'theta true');
% 
% subplot(3, 1, 3);
% plot(t, rad2deg(z_measurement_hist(3, :)), 'r:', t, rad2deg(z_true_hist(3, :)), 'k' , t, rad2deg(state_filter_hist(3, :)), 'g--');
% xlabel('[sec]')
% ylabel('[deg/s]')
% title('r');
% legend('r measurement', 'r true');

% Plot Model Probabilities
figure(4);
plot(t, mu_hist(1,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'CV Model Probability');
hold on;
plot(t, mu_hist(2,:), 'r-', 'LineWidth', 1.5, 'DisplayName', 'CA Model Probability');
xlabel('Time (sec)');
ylabel('Model Probability');
title('IMM Model Probabilities Over Time');
legend('show');
grid on;


% Plot IMM Estimates
figure(5);
subplot(3,1,1);
plot(t, X_IMM_hist(1, :), 'k-', t, target_history(1, :), 'r--');
xlabel('[sec]');
ylabel('[mm]');
title('IMM X Position');
legend('IMM', 'Target');

subplot(3,1,2);
plot(t, X_IMM_hist(2, :), 'k-', t, target_history(2, :), 'r--');
xlabel('[sec]');
ylabel('[mm]');
title('IMM Y Position');
legend('IMM', 'Target');

subplot(3,1,3);
plot(t, rad2deg(X_IMM_hist(5, :)), 'k-', t, rad2deg(target_history(5, :)), 'r--');
xlabel('[sec]');
ylabel('[deg]');
title('IMM Yaw Angle (psi)');
legend('IMM', 'Target');


function log_p = log_mvnpdf(x, mu, Sigma)
    k = length(x);
    term1 = -0.5 * (x - mu)' * (Sigma \ (x - mu));
    term2 = -0.5 * k * log(2 * pi);
    term3 = -0.5 * log(det(Sigma));
    log_p = term1 + term2 + term3;
end
