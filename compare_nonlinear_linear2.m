%% HANul 로켓 선형화 시뮬레이션
% Last update: 2025-05-06

%% Initialize Matlab
clc;
clear all;
% close all;

% Add Library directory (경로 수정 필요)
addpath(genpath('C:\Users\USER\Desktop\hanul_GNC'));

% ===== 파라미터 로드 =====
xmlFilePath = 'rocket.xml';
rocketParams = extractRocketParams(xmlFilePath);
flatRocketParams = flattenRocketStructure(rocketParams);
motorCsvPath = 'AeroTech_M2400T.csv';
force_moment_path = 'csv_Force_moment_axiseq.csv';

% 초기 파라미터 설정
params = vehicle_params(flatRocketParams, 0, motorCsvPath, true);

% ===== 초기 상태 설정 =====
pos = [ 0, 0, 0 ]';                        % 위치 [m] - 관성 좌표계
V_B = [ 0, 0, 0 ]';                        % 속도 [m/s] - 동체 좌표계
att_euler = [ deg2rad(50), deg2rad(80), deg2rad(0) ]; % 자세각 [rad]
att_quat = GetQUAT(att_euler(3), att_euler(2), att_euler(1))'; % 쿼터니언
omega = [ 0, 0, 0 ]';                      % 각속도 [rad/s]

X = [ pos; V_B; att_quat; omega ];         % 상태 벡터
Qf = diag(zeros(6,1));                     % 노이즈 벡터

% ===== 시뮬레이션 설정 =====
Sim_Loop_Hz = 400;                         % 시뮬레이션 주파수 [Hz]
Sim_time = 30;                             % 시뮬레이션 시간 [s]
attitude_Control_Hz = 20;                  % 자세 제어 주파수 [Hz]
dt_sim = 1 / Sim_Loop_Hz;                  % 시간 스텝 [s]
t = (0 : Sim_time / dt_sim) * dt_sim;      % 시간 벡터
num_steps = length(t);                     % 총 스텝 수

% ===== 기록용 변수 초기화 =====
% 상태 기록
state_history = zeros(length(X), num_steps);
state_history(:,1) = X;
X_lin_history = zeros(length(X), num_steps);
X_lin_history(:,1) = X;

% 제어 및 추가 변수 기록
control_input_history = zeros(8, num_steps);
Thrust_mass_history = zeros(4, num_steps);
alpha_tot_history = zeros(1, num_steps);
phi_A_history = zeros(1, num_steps);
F_aero_history = zeros(3, num_steps);
M_aero_history = zeros(3, num_steps);
moment_coupling_history = zeros(3, num_steps);

% 선형화 관련 기록
lin_error_history = zeros(length(X), num_steps);
Ad_history = zeros(13, 13, num_steps);
Bd_history = zeros(13, 8, num_steps);
eig_history = zeros(13, num_steps);

% 제어 간격 설정
attitude_angle_interval = Sim_Loop_Hz / attitude_Control_Hz;

% 진행 상태 표시
hWait = waitbar(0, '시뮬레이션 진행 중...', 'Name', '진행 상태');
update_interval = 100;

try
    % ===== 메인 시뮬레이션 루프 =====
    for i = 1:num_steps-1
        % 1. 현재 시간 및 파라미터 업데이트
        current_time = (i-1) * dt_sim;
        params = vehicle_params(flatRocketParams, current_time, motorCsvPath, false);
        
        % 2. 제어 명령 생성 및 할당
        Command_Vector = generate_commands_HANul(i, Sim_Loop_Hz);
        
        % 제어기 주기적 실행
        if mod(i-1, attitude_angle_interval) == 0
            U = Controller_PPID_Argument(X, Command_Vector, dt_sim, params);
        end 
        
        % 제어 명령 할당
        [final_cmd, debug_cmd] = Control_Allocator_RCS(U, params);
        
        % 3. 선형화 수행 (새로운 래퍼 함수 사용)
        [Ad_i, Bd_i] = linearize_rocket(X, final_cmd, params, dt_sim);
        
        % 선형화 결과 저장
        Ad_history(:,:,i) = Ad_i;
        Bd_history(:,:,i) = Bd_i;
        
        % 4. 선형화 모델을 사용한 다음 상태 예측
        X_lin_next = Ad_i * X + Bd_i * final_cmd;
        X_lin_history(:,i+1) = X_lin_next;
        
        % 5. 비선형 동역학 적분 (SRK4)
        [X_next_nonlin, Thrust_mass, alpha_tot, phi_A, F_aero, M_aero, moment_coupling] = ...
            srk4(@vehicle_dynamics_HANul, X, final_cmd, Qf, dt_sim, params, current_time, dt_sim);
        
        % 6. 선형화 예측과 비선형 결과 비교
        lin_error = X_next_nonlin - X_lin_next;
        lin_error_history(:,i+1) = lin_error;
        
        % 7. 상태 업데이트 및 데이터 기록
        X = X_next_nonlin;
        state_history(:,i+1) = X;
        control_input_history(:,i) = final_cmd;
        Thrust_mass_history(:,i) = Thrust_mass;
        alpha_tot_history(i) = alpha_tot;
        phi_A_history(i) = phi_A;
        F_aero_history(:,i) = F_aero;
        M_aero_history(:,i) = M_aero;
        moment_coupling_history(:,i) = moment_coupling;
        
        % 8. 진행 상태 표시
        if mod(i, update_interval) == 0 || i == num_steps-1
            progress = i / (num_steps-1);
            waitbar(progress, hWait, sprintf('진행 중: %.2f%%', progress * 100));
        end
    end
    
    % 마지막 상태 기록
    control_input_history(:,num_steps) = final_cmd;
    
    % 진행 상태 창 닫기
    close(hWait);
    
catch ME
    % 오류 발생 시 진행 상태 창 닫고 오류 출력
    close(hWait);
    rethrow(ME);
end

%% 결과 분석 및 시각화

% 1. 오차 통계 계산
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

% 2. 위치 및 속도 비교 그래프
figure(1);
subplot(3,2,1);
plot(t, state_history(1,:), 'b-', t, X_lin_history(1,:), 'r--');
title('X 위치'); xlabel('시간 [s]'); ylabel('위치 [m]');
legend('비선형 (SRK4)', '선형화 모델');
grid on;
hold on

subplot(3,2,2);
plot(t, state_history(4,:), 'b-', t, X_lin_history(4,:), 'r--');
title('u 속도'); xlabel('시간 [s]'); ylabel('속도 [m/s]');
legend('비선형 (SRK4)', '선형화 모델');
grid on;
hold on

subplot(3,2,3);
plot(t, state_history(2,:), 'b-', t, X_lin_history(2,:), 'r--');
title('Y 위치'); xlabel('시간 [s]'); ylabel('위치 [m]');
legend('비선형 (SRK4)', '선형화 모델');
grid on;
hold on

subplot(3,2,4);
plot(t, state_history(5,:), 'b-', t, X_lin_history(5,:), 'r--');
title('v 속도'); xlabel('시간 [s]'); ylabel('속도 [m/s]');
legend('비선형 (SRK4)', '선형화 모델');
grid on;
hold on

subplot(3,2,5);
plot(t, -state_history(3,:), 'b-', t, -X_lin_history(3,:), 'r--');
title('고도 (Z)'); xlabel('시간 [s]'); ylabel('고도 [m]');
legend('비선형 (SRK4)', '선형화 모델');
grid on;

subplot(3,2,6);
plot(t, state_history(6,:), 'b-', t, X_lin_history(6,:), 'r--');
title('w 속도'); xlabel('시간 [s]'); ylabel('속도 [m/s]');
legend('비선형 (SRK4)', '선형화 모델');
grid on;
hold on

% 3. 각속도 비교 그래프
figure(2);
subplot(3,1,1);
plot(t, state_history(11,:), 'b-', t, X_lin_history(11,:), 'r--');
title('롤 각속도 (p)'); xlabel('시간 [s]'); ylabel('각속도 [rad/s]');
legend('비선형 (SRK4)', '선형화 모델');
grid on;
hold on

subplot(3,1,2);
plot(t, state_history(12,:), 'b-', t, X_lin_history(12,:), 'r--');
title('피치 각속도 (q)'); xlabel('시간 [s]'); ylabel('각속도 [rad/s]');
legend('비선형 (SRK4)', '선형화 모델');
grid on;
hold on

subplot(3,1,3);
plot(t, state_history(13,:), 'b-', t, X_lin_history(13,:), 'r--');
title('요 각속도 (r)'); xlabel('시간 [s]'); ylabel('각속도 [rad/s]');
legend('비선형 (SRK4)', '선형화 모델');
grid on;
hold on

% 5. 쿼터니언 비교 그래프
figure(4);
subplot(4,1,1);
plot(t, state_history(7,:), 'b-', t, X_lin_history(7,:), 'r--');
title('쿼터니언 q0'); xlabel('시간 [s]'); ylabel('q0');
legend('비선형 (SRK4)', '선형화 모델');
grid on;
hold on

subplot(4,1,2);
plot(t, state_history(8,:), 'b-', t, X_lin_history(8,:), 'r--');
title('쿼터니언 q1'); xlabel('시간 [s]'); ylabel('q1');
legend('비선형 (SRK4)', '선형화 모델');
grid on;
hold on

subplot(4,1,3);
plot(t, state_history(9,:), 'b-', t, X_lin_history(9,:), 'r--');
title('쿼터니언 q2'); xlabel('시간 [s]'); ylabel('q2');
legend('비선형 (SRK4)', '선형화 모델');
grid on;
hold on

subplot(4,1,4);
plot(t, state_history(10,:), 'b-', t, X_lin_history(10,:), 'r--');
title('쿼터니언 q3'); xlabel('시간 [s]'); ylabel('q3');
legend('비선형 (SRK4)', '선형화 모델');
grid on;
hold on


% 6. 오일러 각 비교 그래프
figure(5);
euler_nonlinear = zeros(3, num_steps);
euler_linear = zeros(3, num_steps);

for i = 1:num_steps
    % 비선형 모델 쿼터니언에서 오일러 각 변환
    quat_nonlin = state_history(7:10,i);
    euler_nonlinear(:,i) = Quat2Euler(quat_nonlin);
    
    % 선형 모델 쿼터니언에서 오일러 각 변환
    quat_lin = X_lin_history(7:10,i);
    euler_linear(:,i) = Quat2Euler(quat_lin);
end

subplot(3,1,1);
plot(t, rad2deg(euler_nonlinear(1,:)), 'b-', t, rad2deg(euler_linear(1,:)), 'r--');
title('롤 각도'); xlabel('시간 [s]'); ylabel('각도 [deg]');
legend('비선형 (SRK4)', '선형화 모델');
grid on;
hold on

subplot(3,1,2);
plot(t, rad2deg(euler_nonlinear(2,:)), 'b-', t, rad2deg(euler_linear(2,:)), 'r--');
title('피치 각도'); xlabel('시간 [s]'); ylabel('각도 [deg]');
legend('비선형 (SRK4)', '선형화 모델');
grid on;
hold on

subplot(3,1,3);
plot(t, rad2deg(euler_nonlinear(3,:)), 'b-', t, rad2deg(euler_linear(3,:)), 'r--');
title('요 각도'); xlabel('시간 [s]'); ylabel('각도 [deg]');
legend('비선형 (SRK4)', '선형화 모델');
grid on;
hold on


% 4. 예측 오차 그래프
figure(6);
subplot(5,1,1);
semilogy(t, lin_error_norm);
title('전체 상태 벡터 예측 오차'); xlabel('시간 [s]'); ylabel('오차 (로그 스케일)');
grid on;
hold on

subplot(5,1,2);
semilogy(t, lin_error_pos_norm);
title('위치 예측 오차'); xlabel('시간 [s]'); ylabel('오차 [m]');
grid on;
hold on

subplot(5,1,3);
semilogy(t, lin_error_vel_norm);
title('속도 예측 오차'); xlabel('시간 [s]'); ylabel('오차 [m/s]');
grid on;
hold on

subplot(5,1,4);
semilogy(t, lin_error_quat_norm);
title('쿼터니언 예측 오차'); xlabel('시간 [s]'); ylabel('오차');
grid on;
hold on

subplot(5,1,5);
semilogy(t, lin_error_omega_norm);
title('각속도 예측 오차'); xlabel('시간 [s]'); ylabel('오차 [rad/s]');
grid on;
hold on


% 7. 통계 정보 출력
fprintf('\n===== 선형화 모델 예측 오차 통계 =====\n');
fprintf('전체 상태 벡터 평균 오차: %f\n', mean(lin_error_norm));
fprintf('위치 평균 오차: %f m\n', mean(lin_error_pos_norm));
fprintf('속도 평균 오차: %f m/s\n', mean(lin_error_vel_norm));
fprintf('쿼터니언 평균 오차: %f\n', mean(lin_error_quat_norm));
fprintf('각속도 평균 오차: %f rad/s\n', mean(lin_error_omega_norm));