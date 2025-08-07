%% HANul 로켓 실시간 선형화 비교 시뮬레이션
% 이 스크립트는 매 시뮬레이션 단계마다 선형화를 수행하고, 
% 비선형 SRK4 통합자 결과와 선형화 예측 결과를 비교합니다.
% Last update: 2025-05-06

%% Initialize Matlab
clc;
clear all;
close all;

% Add Library directory (develope version path code)
addpath(genpath('C:\Users\USER\Desktop\hanul_GNC'));

% ===== Vehicle Geometry & Mass Properties load =====
xmlFilePath = 'rocket.xml';
rocketParams = extractRocketParams(xmlFilePath);
flatRocketParams = flattenRocketStructure(rocketParams);

% ===== Motor data Load =====
motorCsvPath = 'AeroTech_M2400T.csv';

% ===== Aerodynamics coeficent data Load =====
force_moment_path = 'csv_Force_moment_axiseq.csv';  % 정적 공력계수 파일

% Simulation Parameters
% 기존 Params_init_HANul() 대신 vehicle_params() 함수 사용
params = vehicle_params(flatRocketParams, 0, motorCsvPath, true); % 초기화 (reset=true)

% Initialize State Vector
pos = [ 0, 0, 0 ]'; % Position XYZ [m] - Inertial frame  
V_B = [ 0, 0, 0 ]'; % Velocity uvw [m/s] - body frame 
att_euler = [ deg2rad(50), deg2rad(87), deg2rad(0) ]; % Attitude angle(Euler: roll, pitch, yaw) [rad] - Inertial frame
att_quat = GetQUAT( att_euler(3), att_euler(2), att_euler(1) )'; % Attitude Quaternion - Inertial frame 
omega = [ 0, 0, 0 ]'; % Angular rates [rad/s^2] - body frame 

X_int_q = [ pos; V_B; att_quat; omega ]; % Initial State Vector (Always initial value vector)
X = X_int_q; % Current State Vector initialize 

RI2B = GetDCM_Euler( att_euler(3), att_euler(2), att_euler(1) );
RB2I = RI2B';

% Simulation Setting
Sim_Loop_Hz = 400;
Sim_time = 30;

attitude_Control_Hz = 20;
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

% 시뮬레이션 루프 전에 히스토리 변수 초기화
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

% 실시간 선형화를 위한 추가 변수 초기화
delta_x = 1e-4;                        % 상태 변수 섭동 크기
delta_u = 1e-4;                        % 제어 입력 섭동 크기
X_lin_history = zeros(length(X), num_steps);  % 선형화 기반 예측 상태
X_lin_history(:,1) = X;                % 첫번째 스텝은 초기값으로 설정
lin_error_history = zeros(length(X), num_steps); % 선형화 예측 오차
Ad_history = zeros(13, 13, num_steps); % 각 스텝의 Ad 행렬 저장
Bd_history = zeros(13, 8, num_steps);  % 각 스텝의 Bd 행렬 저장
eig_history = zeros(13, num_steps);    % 각 스텝의 고유값(연속시간) 저장
max_eig_real_history = zeros(1, num_steps); % 최대 실수부 저장

% Controller interval Setting
attitude_angle_interval = Sim_Loop_Hz / attitude_Control_Hz; 

% Initialize simulation time
real_time = 0;

% Initialize Waitbar
hWait = waitbar(0, '시뮬레이션 진행 중...', 'Name', '진행 상태');
% Set waitbar update interval to reduce overhead
update_interval = 100; % Update every 100 steps

try
    % Main simulation loop
    for i = 1:num_steps-1  % 마지막 스텝을 제외하고 반복 (예측 비교를 위해)
        % 현재 시뮬레이션 시간
        current_time = (i-1) * dt_sim;
        
        % 현재 시간에 따른 파라미터 업데이트 (vehicle_params 함수 호출)
        params = vehicle_params(flatRocketParams, current_time, motorCsvPath, false);
        
        % 명령 생성
        [ Command_Vector ] = generate_commands_HANul(i, Sim_Loop_Hz);

        % 제어기 실행 (attitude_Control_Hz 주기로)
        if mod(i-1, attitude_angle_interval) == 0
            U = Controller_PPID_Argument(X, Command_Vector, dt_sim, params);
        end 

        % 제어 명령 할당
        [ final_cmd, debug_cmd ] = Control_Allocator_RCS(U, params);
        
        % 여기서 실시간 선형화 수행 - 수치적 자코비안 계산
        % Ad(x,u) 행렬 계산 - 상태 변수 섭동
        Ad_i = eye(13);  % 이산시간 선형화 행렬 초기화
        
        for j = 1:13
            % 상태 변수 j에 작은 변화 적용
            X_perturb = X;
            X_perturb(j) = X_perturb(j) + delta_x;
            
            % 쿼터니언 특별 처리 (정규화)
            if j >= 7 && j <= 10
                X_perturb(7:10) = X_perturb(7:10) / norm(X_perturb(7:10));
            end
            
            % 섭동된 상태로 dynamics 함수 호출
            [X_next_perturb, ~, ~, ~, ~, ~, ~] = vehicle_dynamics_HANul(X_perturb, final_cmd, Qf, dt_sim, params, current_time);
            
            % 원래 상태로 dynamics 함수 호출
            [X_next, ~, ~, ~, ~, ~, ~] = vehicle_dynamics_HANul(X, final_cmd, Qf, dt_sim, params, current_time);
            
            % 수치적 자코비안 계산 (전진 차분법)
            % 주의: 이산시간 행렬을 계산하기 위해 Forward Euler 근사 사용
            dX = (X_next_perturb - X_next) / delta_x;
            Ad_i(:,j) = dX * dt_sim + (j==1:13)';  % Ad = I + A*dt
        end
        
        % Bd(x,u) 행렬 계산 - 입력 변수 섭동
        Bd_i = zeros(13, 8);  % 이산시간 입력 행렬 초기화
        
        for j = 1:8
            % 제어 입력 j에 작은 변화 적용
            U_perturb = final_cmd;
            U_perturb(j) = U_perturb(j) + delta_u;
            
            % 섭동된 입력으로 dynamics 함수 호출
            [X_next_perturb, ~, ~, ~, ~, ~, ~] = vehicle_dynamics_HANul(X, U_perturb, Qf, dt_sim, params, current_time);
            
            % 원래 입력으로 dynamics 함수 호출
            [X_next, ~, ~, ~, ~, ~, ~] = vehicle_dynamics_HANul(X, final_cmd, Qf, dt_sim, params, current_time);
            
            % 수치적 자코비안 계산 (전진 차분법)
            dX = (X_next_perturb - X_next) / delta_u;
            Bd_i(:,j) = dX * dt_sim;  % Bd = B*dt
        end
        
        % 선형화 행렬 저장
        Ad_history(:,:,i) = Ad_i;
        Bd_history(:,:,i) = Bd_i;
        
        % 연속시간 시스템 행렬로 변환하여 고유값 계산
        try
            % A = (Ad-I)/dt 근사
            A_i = (Ad_i - eye(13)) / dt_sim;
            eig_i = eig(A_i);
            eig_history(:,i) = eig_i;
            max_eig_real_history(i) = max(real(eig_i));
        catch
            % 고유값 계산 실패 시 NaN으로 설정
            eig_history(:,i) = NaN(13,1);
            max_eig_real_history(i) = NaN;
        end
        
        % 선형화 모델을 사용한 다음 상태 예측
        % X(k+1) = Ad*X(k) + Bd*u(k)
        X_lin_next = Ad_i * X + Bd_i * final_cmd;
        
        % 비선형 동역학 적분 - 업데이트된 파라미터 전달
        [ X_next_nonlin, Thrust_mass, alpha_tot, phi_A, F_aero, M_aero, moment_coupling ] = srk4(@vehicle_dynamics_HANul, X, final_cmd, Qf, dt_sim, params, current_time, dt_sim);
        
        % 선형화 예측과 비선형 SRK4 결과 비교
        lin_error = X_next_nonlin - X_lin_next;
        lin_error_history(:,i+1) = lin_error;  % 다음 스텝의 오차 저장
        
        % 선형화 예측 상태 저장
        X_lin_history(:,i+1) = X_lin_next;
        
        % 비선형 통합 결과로 상태 업데이트
        X = X_next_nonlin;
        
        % 자세 변환 (쿼터니언 -> 오일러)
        quat_now = X(7:10);
        Euler = Quat2Euler(quat_now');
        X_Euler = X;
        X_Euler(7:9) = rad2deg(Euler);
        X_Euler(10:12) = X(11:13);

        quat_error = q2q_mult(quat_now, inv_q(Command_Vector'));
        quat_error_history(:, i) = quat_error;
        quat_error_magnitude(i) = 1 - abs(quat_error(1));


        % 데이터 기록
        state_history(:, i) = X;
        state_history2(:, i) = X_Euler;
        Log_save(:, i) = [X_Euler(7:9); X(1:3)];
        debug_q(:, i) = quat_now;
        Thrust_mass_history(:, i) = Thrust_mass;

        % Store control inputs
        control_input_history(:, i) = [final_cmd(1:4); final_cmd(5:8)];

        alpha_tot_history(i) = alpha_tot;
        target_quat_history(:, i) = Command_Vector;
        euler_command_history(:, i) = rad2deg(Quat2Euler(Command_Vector));

        phi_A_history(i) = phi_A;
        F_aero_history(:,i) = F_aero;
        M_aero_history(:,i) = M_aero;
        MRP_history(:,i) = [params.vehicle.MRP_x; params.vehicle.MRP_y; params.vehicle.MRP_z];

        moment_coupling_history(:,i) = moment_coupling;
        moment_coupling_magnitude(i) = norm(moment_coupling);

        real_time = real_time + dt_sim;

        % 진행 상태 업데이트 (주기적)
        if mod(i, update_interval) == 0 || i == num_steps-1
            progress = i / (num_steps-1);
            waitbar(progress, hWait, sprintf('진행 중: %.2f%%', progress * 100));
        end
    end

    % 마지막 스텝 데이터 기록 (예측 비교를 제외)
    state_history(:, num_steps) = X;
    
    % 선형화 오차 통계 계산
    lin_error_norm = zeros(num_steps, 1);
    lin_error_pos_norm = zeros(num_steps, 1);
    lin_error_vel_norm = zeros(num_steps, 1);
    lin_error_quat_norm = zeros(num_steps, 1);
    lin_error_omega_norm = zeros(num_steps, 1);
    
    for i = 1:num_steps
        lin_error_norm(i) = norm(lin_error_history(:,i));
        lin_error_pos_norm(i) = norm(lin_error_history(1:3,i));
        lin_error_vel_norm(i) = norm(lin_error_history(4:6,i));
        lin_error_quat_norm(i) = norm(lin_error_history(7:10,i));
        lin_error_omega_norm(i) = norm(lin_error_history(11:13,i));
    end

    % 완료 후 Waitbar 닫기
    close(hWait);

catch ME
    % 오류 발생 시 Waitbar 닫기
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


%% Post-processing and plotting

% Plot Position and Velocity
figure(1);
titles = {'X Position', 'Y Position', 'Altitude (Z)', 'u Velocity', 'v Velocity', 'w Velocity'};
ylabels = {'Position [m]', 'Position [m]', 'Altitude [m]', 'Velocity [m/s]', 'Velocity [m/s]', 'Velocity [m/s]'};

for k = 1:6
    subplot(2, 3, k);
    switch k
        case 1  % X Position
            plot(t, state_history(1, :), 'b-', 'LineWidth', 1.5);
            hold on;
            plot(t, X_lin_history(1, :), 'r--', 'LineWidth', 1);
        case 2  % Y Position
            plot(t, state_history(2, :), 'b-', 'LineWidth', 1.5);
            hold on;
            plot(t, X_lin_history(2, :), 'r--', 'LineWidth', 1);
        case 3  % Altitude (Z)
            plot(t, -state_history(3, :), 'b-', 'LineWidth', 1.5);  % Altitude is negative of Z position
            hold on;
            plot(t, -X_lin_history(3, :), 'r--', 'LineWidth', 1);
        case 4  % u Velocity
            plot(t, state_history(4, :), 'b-', 'LineWidth', 1.5);
            hold on;
            plot(t, X_lin_history(4, :), 'r--', 'LineWidth', 1);
        case 5  % v Velocity
            plot(t, state_history(5, :), 'b-', 'LineWidth', 1.5);
            hold on;
            plot(t, X_lin_history(5, :), 'r--', 'LineWidth', 1);
        case 6  % w Velocity
            plot(t, state_history(6, :), 'b-', 'LineWidth', 1.5);
            hold on;
            plot(t, X_lin_history(6, :), 'r--', 'LineWidth', 1);
    end
    xlabel('Time [s]');
    ylabel(ylabels{k});
    title(titles{k});
    grid on;
    legend('비선형 모델(SRK4)', '선형화 모델');

    % Add Apogee line
    if ~isnan(apogee_time)
        line([apogee_time apogee_time], ylim, 'Color', 'k', 'LineStyle', '--', 'DisplayName', 'Apogee');
    end
end

% 각속도 비교 그래프
figure(2);
titles = {'Roll Rate p', 'Pitch Rate q', 'Yaw Rate r'};
ylabels = {'Angular Rate [rad/s]', 'Angular Rate [rad/s]', 'Angular Rate [rad/s]'};

for k = 1:3
    subplot(3, 1, k);
    plot(t, state_history(k+10, :), 'b-', 'LineWidth', 1.5);
    hold on;
    plot(t, X_lin_history(k+10, :), 'r--', 'LineWidth', 1);
    xlabel('Time [s]');
    ylabel(ylabels{k});
    title(titles{k});
    grid on;
    legend('비선형 모델(SRK4)', '선형화 모델');
end

% 선형화 예측 오차 그래프
figure(3);
sgtitle('선형화 예측 오차');

subplot(5, 1, 1);
plot(t, lin_error_norm);
title('전체 상태 벡터 예측 오차 (놈)');
xlabel('Time [s]');
ylabel('Error Norm');
grid on;

subplot(5, 1, 2);
plot(t, lin_error_pos_norm);
title('위치 예측 오차 (놈)');
xlabel('Time [s]');
ylabel('Position Error [m]');
grid on;

subplot(5, 1, 3);
plot(t, lin_error_vel_norm);
title('속도 예측 오차 (놈)');
xlabel('Time [s]');
ylabel('Velocity Error [m/s]');
grid on;

subplot(5, 1, 4);
plot(t, lin_error_quat_norm);
title('쿼터니언 예측 오차 (놈)');
xlabel('Time [s]');
ylabel('Quaternion Error');
grid on;

subplot(5, 1, 5);
plot(t, lin_error_omega_norm);
title('각속도 예측 오차 (놈)');
xlabel('Time [s]');
ylabel('Angular Rate Error [rad/s]');
grid on;

% 고유값 분석 그래프
figure(4);
sgtitle('시스템 고유값 분석');

subplot(2, 1, 1);
real_parts = zeros(13, num_steps-1);
imag_parts = zeros(13, num_steps-1);

for i = 1:num_steps-1
    if ~any(isnan(eig_history(:,i)))
        real_parts(:,i) = real(eig_history(:,i));
        imag_parts(:,i) = imag(eig_history(:,i));
    end
end

% 첫 번째는 시간 경과에 따른 고유값 실수부
plot(t(1:num_steps-1), real_parts);
title('시간에 따른 고유값 실수부 변화');
xlabel('Time [s]');
ylabel('Real part');
grid on;
yline(0, 'r--', 'LineWidth', 2);  % 안정성 경계선

subplot(2, 1, 2);
plot(t(1:num_steps-1), max_eig_real_history(1:num_steps-1));
title('최대 고유값 실수부');
xlabel('Time [s]');
ylabel('Max real part');
grid on;
yline(0, 'r--', 'LineWidth', 2);  % 안정성 경계선

% 마지막 시뮬레이션 스텝에서의 고유값 산점도
figure(5);
last_valid_idx = find(~isnan(eig_history(1,:)), 1, 'last');
if ~isempty(last_valid_idx)
    scatter(real(eig_history(:,last_valid_idx)), imag(eig_history(:,last_valid_idx)), 50, 'filled');
    title('마지막 유효 스텝에서의 시스템 고유값');
    xlabel('Real part');
    ylabel('Imaginary part');
    grid on;
    xline(0, 'r--', 'LineWidth', 2);  % 안정성 경계선
end

% 비선형 모델과 선형화 모델의 오차를 시각화하는 추가 그래프
figure(6);
sgtitle('비선형 모델과 선형화 모델 비교 - 오차 분석');

% 상태별 상대 오차 계산
rel_err_x = abs((state_history(1,:) - X_lin_history(1,:)) ./ max(abs(state_history(1,:)), 1e-6)) * 100;
rel_err_y = abs((state_history(2,:) - X_lin_history(2,:)) ./ max(abs(state_history(2,:)), 1e-6)) * 100;
rel_err_z = abs((state_history(3,:) - X_lin_history(3,:)) ./ max(abs(state_history(3,:)), 1e-6)) * 100;
rel_err_u = abs((state_history(4,:) - X_lin_history(4,:)) ./ max(abs(state_history(4,:)), 1e-6)) * 100;
rel_err_v = abs((state_history(5,:) - X_lin_history(5,:)) ./ max(abs(state_history(5,:)), 1e-6)) * 100;
rel_err_w = abs((state_history(6,:) - X_lin_history(6,:)) ./ max(abs(state_history(6,:)), 1e-6)) * 100;
rel_err_p = abs((state_history(11,:) - X_lin_history(11,:)) ./ max(abs(state_history(11,:)), 1e-6)) * 100;
rel_err_q = abs((state_history(12,:) - X_lin_history(12,:)) ./ max(abs(state_history(12,:)), 1e-6)) * 100;
rel_err_r = abs((state_history(13,:) - X_lin_history(13,:)) ./ max(abs(state_history(13,:)), 1e-6)) * 100;

% 위치 상대 오차
subplot(3, 3, 1);
semilogy(t, rel_err_x); % 로그 스케일 사용
title('X 위치 상대 오차');
xlabel('Time [s]');
ylabel('Relative Error [%]');
grid on;

subplot(3, 3, 2);
semilogy(t, rel_err_y);
title('Y 위치 상대 오차');
xlabel('Time [s]');
ylabel('Relative Error [%]');
grid on;

subplot(3, 3, 3);
semilogy(t, rel_err_z);
title('Z 위치 상대 오차');
xlabel('Time [s]');
ylabel('Relative Error [%]');
grid on;

% 속도 상대 오차
subplot(3, 3, 4);
semilogy(t, rel_err_u);
title('u 속도 상대 오차');
xlabel('Time [s]');
ylabel('Relative Error [%]');
grid on;

subplot(3, 3, 5);
semilogy(t, rel_err_v);
title('v 속도 상대 오차');
xlabel('Time [s]');
ylabel('Relative Error [%]');
grid on;

subplot(3, 3, 6);
semilogy(t, rel_err_w);
title('w 속도 상대 오차');
xlabel('Time [s]');
ylabel('Relative Error [%]');
grid on;

% 각속도 상대 오차
subplot(3, 3, 7);
semilogy(t, rel_err_p);
title('p 각속도 상대 오차');
xlabel('Time [s]');
ylabel('Relative Error [%]');
grid on;

subplot(3, 3, 8);
semilogy(t, rel_err_q);
title('q 각속도 상대 오차');
xlabel('Time [s]');
ylabel('Relative Error [%]');
grid on;

subplot(3, 3, 9);
semilogy(t, rel_err_r);
title('r 각속도 상대 오차');
xlabel('Time [s]');
ylabel('Relative Error [%]');
grid on;

% 오차 통계 요약
disp('===== 선형화 모델 예측 오차 통계 =====');
disp(['전체 시뮬레이션 기간 동안의 평균 오차 (놈): ' num2str(mean(lin_error_norm))]);
disp(['위치 평균 오차 (놈): ' num2str(mean(lin_error_pos_norm))]);
disp(['속도 평균 오차 (놈): ' num2str(mean(lin_error_vel_norm))]);
disp(['쿼터니언 평균 오차 (놈): ' num2str(mean(lin_error_quat_norm))]);
disp(['각속도 평균 오차 (놈): ' num2str(mean(lin_error_omega_norm))]);

% 고유값 통계
stable_steps = sum(max_eig_real_history < 0);
unstable_steps = sum(max_eig_real_history >= 0);
disp('===== 시스템 안정성 분석 =====');
disp(['안정한 스텝 수 (모든 고유값 실수부 < 0): ' num2str(stable_steps)]);
disp(['불안정한 스텝 수 (최소 하나의 고유값 실수부 >= 0): ' num2str(unstable_steps)]);
disp(['불안정 비율: ' num2str(unstable_steps / (stable_steps + unstable_steps) * 100) '%']);