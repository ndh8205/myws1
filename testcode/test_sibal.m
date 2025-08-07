%% HANul 로켓 선형화 방법 비교 및 MPC 시뮬레이션
% 두 가지 선형화 방법 비교와 MPC 예측 성능 평가
% Last update: 2025-05-06

%% Initialize Matlab
clc;
clear all;
close all;

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
att_euler = [ deg2rad(50), deg2rad(87), deg2rad(0) ]; % 자세각 [rad]
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

% ===== MPC 설정 =====
MPC_horizon = 20;                          % 예측 지평선
MPC_control_Hz = attitude_Control_Hz;      % MPC 제어 주파수 [Hz]
MPC_control_interval = Sim_Loop_Hz / MPC_control_Hz; % 제어 간격
MPC_dt = 1 / MPC_control_Hz;              % MPC 시간 스텝 [s]

% MPC 가중치 행렬
% 상태 가중치
Q_att = diag([10, 10, 10, 10]);           % 쿼터니언 가중치
Q_omega = diag([5, 5, 5]);                % 각속도 가중치
Q_pos = diag([1, 1, 1]);                  % 위치 가중치 
Q_vel = diag([1, 1, 1]);                  % 속도 가중치
Q_full = blkdiag(Q_pos, Q_vel, Q_att, Q_omega); % 전체 상태 가중치

% 제어 입력 가중치
R_canard = diag([0.1, 0.1, 0.1, 0.1]);    % 카나드 제어 가중치
R_RCS = diag([0.5, 0.5, 0.5, 0.5]);       % RCS 제어 가중치
R = blkdiag(R_canard, R_RCS);             % 전체 제어 가중치

% 제어 입력 제약 조건
canard_limit = deg2rad(20);               % 카나드 각도 제한 [rad]
RCS_limit = 1;                            % RCS 제어 제한 (0-1)

u_min = [-canard_limit*ones(4,1); zeros(4,1)];
u_max = [canard_limit*ones(4,1); RCS_limit*ones(4,1)];

% ===== 기록용 변수 초기화 =====
% 상태 기록
state_history = zeros(length(X), num_steps);
state_history(:,1) = X;

% 각 선형화 모델에 따른 예측 상태
X_lin1_history = zeros(length(X), num_steps);  % linearize_rocket 함수 사용
X_lin1_history(:,1) = X;
X_lin2_history = zeros(length(X), num_steps);  % linearize_HANul_quaternion 함수 사용 
X_lin2_history(:,1) = X;

% MPC 예측 궤적 기록 (Nx13xMPC_horizon 배열)
MPC_pred1_history = zeros(num_steps, 13, MPC_horizon+1);  % linearize_rocket 함수 사용
MPC_pred2_history = zeros(num_steps, 13, MPC_horizon+1);  % linearize_HANul_quaternion 함수 사용

% 제어 및 추가 변수 기록
control_input_history = zeros(8, num_steps);
mpc_control1_history = zeros(8, num_steps);    % linearize_rocket 기반 MPC 제어
mpc_control2_history = zeros(8, num_steps);    % linearize_HANul_quaternion 기반 MPC 제어

% 선형화 행렬 기록
Ad1_history = zeros(13, 13, num_steps);        % linearize_rocket의 Ad
Bd1_history = zeros(13, 8, num_steps);         % linearize_rocket의 Bd
Ad2_full_history = zeros(13, 13, num_steps);   % linearize_HANul_quaternion의 Ad (확장됨)
Bd2_full_history = zeros(13, 8, num_steps);    % linearize_HANul_quaternion의 Bd (확장됨)

% 예측 오차 기록
lin1_error_history = zeros(length(X), num_steps);
lin2_error_history = zeros(length(X), num_steps);
mpc1_error_history = zeros(length(X), num_steps);
mpc2_error_history = zeros(length(X), num_steps);

% 선형화 행렬 비교 지표
matrix_norm_diff_history = zeros(1, num_steps);  % 두 행렬의 차이 (Frobenius norm)
eigenvalue_diff_history = zeros(1, num_steps);   % 고유값 차이의 최댓값

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
        
        % 2. 명령 벡터 생성 (참조 궤적)
        Command_Vector = generate_commands_HANul(i, Sim_Loop_Hz);
        
        % 3. PPID 제어기를 사용한 기본 제어 입력 계산 (MPC 비교용)
        if mod(i-1, attitude_angle_interval) == 0
            U_PPID = Controller_PPID_Argument(X, Command_Vector, dt_sim, params);
            [final_cmd_PPID, ~] = Control_Allocator_RCS(U_PPID, params);
        end
        
        % 4. 두 가지 선형화 방법 실행 및 비교
        % 4.1 linearize_rocket 함수 사용 (전체 13차원 상태)
        [Ad1, Bd1, A1, B1, eig1] = linearize_rocket(X, final_cmd_PPID, params, dt_sim);
        
        % 4.2 linearize_HANul_quaternion 함수 사용 (7차원 자세 상태)
        [Ad2_att, Bd2_att] = linearize_HANul_quaternion(X, final_cmd_PPID, params, dt_sim);
        
        % 4.3 linearize_HANul_quaternion의 결과를 전체 상태 차원으로 확장
        % 7x7 자세 행렬을 13x13 전체 행렬로 확장 (위치/속도는 단순 적분 관계 사용)
        Ad2_full = eye(13);
        Bd2_full = zeros(13, 8);
        
        % 위치, 속도 부분 행렬 설정 (단순 적분)
        Ad2_full(1:3, 1:3) = eye(3);           % 위치-위치
        Ad2_full(1:3, 4:6) = eye(3) * dt_sim;  % 위치-속도
        Ad2_full(4:6, 4:6) = eye(3);           % 속도-속도
        
        % 자세 부분 행렬 복사
        Ad2_full(7:13, 7:13) = Ad2_att;
        Bd2_full(7:13, :) = Bd2_att;
        
        % 행렬 저장
        Ad1_history(:,:,i) = Ad1;
        Bd1_history(:,:,i) = Bd1;
        Ad2_full_history(:,:,i) = Ad2_full;
        Bd2_full_history(:,:,i) = Bd2_full;
        
        % 행렬 비교 지표 계산
        matrix_norm_diff = norm(Ad1 - Ad2_full, 'fro') / norm(Ad1, 'fro');
        matrix_norm_diff_history(i) = matrix_norm_diff;
        
        % 고유값 비교
        eig2_full = eig(Ad2_full);
        if ~isempty(eig1) && ~isempty(eig2_full)
            eig_diff = abs(sort(eig1) - sort(eig2_full));
            eigenvalue_diff_history(i) = max(eig_diff);
        end
        
        % 5. MPC 최적화 (제어 주기마다 실행)
        if mod(i-1, MPC_control_interval) == 0
            % 5.1 linearize_rocket 기반 MPC
            % MPC 예측 궤적 초기화
            X_mpc1_pred = zeros(13, MPC_horizon+1);
            X_mpc1_pred(:,1) = X;
            
            % MPC 제약 조건 설정
            u_prev = final_cmd_PPID;  % 이전 제어 입력
            
            % MPC 비용 함수 및 제약 조건을 사용한 최적화 
            % 실제 MPC 구현을 위한 준비 (간소화된 버전)
            % 참조 상태/입력 설정
            X_ref = zeros(13, 1);
            X_ref(7) = 1;  % 목표 쿼터니언 (q0=1, q1=q2=q3=0)
            
            % 실제 MPC에서는 solve_MPC 함수로 최적화
            % u_opt1 = solve_MPC(X, Ad1, Bd1, Q_full, R, MPC_horizon, u_min, u_max, u_prev, X_ref);
            % 여기서는 간단한 롤링 최적화 시뮬레이션
            u_opt1 = u_prev;
            
            % 예측 지평선에 대한 시뮬레이션
            for j = 1:MPC_horizon
                % 제어 입력에 약간의 무작위성 추가 (실제 MPC는 최적화 사용)
                u_opt1 = min(max(u_opt1 + 0.01*randn(8,1), u_min), u_max);
                
                % 선형 모델을 사용한 다음 상태 예측
                X_mpc1_pred(:,j+1) = Ad1 * X_mpc1_pred(:,j) + Bd1 * u_opt1;
                
                % 쿼터니언 정규화 (중요: MPC 안정성 향상)
                X_mpc1_pred(7:10,j+1) = X_mpc1_pred(7:10,j+1) / norm(X_mpc1_pred(7:10,j+1));
            end
            
            % MPC 예측 궤적 저장 (차원 문제 해결)
            for j = 1:MPC_horizon+1
                MPC_pred1_history(i,:,j) = X_mpc1_pred(:,j);
            end
            
            % 첫 번째 제어 입력 적용
            mpc_control1_history(:,i) = u_opt1;
            
            % 5.2 linearize_HANul_quaternion 기반 MPC
            % MPC 예측 궤적 초기화
            X_mpc2_pred = zeros(13, MPC_horizon+1);
            X_mpc2_pred(:,1) = X;
            
            % MPC 최적화 (간소화된 버전)
            u_opt2 = u_prev;
            
            % 예측 지평선에 대한 시뮬레이션
            for j = 1:MPC_horizon
                % 제어 입력에 약간의 변화 추가 (실제 MPC는 최적화 사용)
                u_opt2 = min(max(u_opt2 + 0.01*randn(8,1), u_min), u_max);
                
                % 선형 모델을 사용한 다음 상태 예측
                X_mpc2_pred(:,j+1) = Ad2_full * X_mpc2_pred(:,j) + Bd2_full * u_opt2;
                
                % 쿼터니언 정규화
                X_mpc2_pred(7:10,j+1) = X_mpc2_pred(7:10,j+1) / norm(X_mpc2_pred(7:10,j+1));
            end
            
            % MPC 예측 궤적 저장 (차원 문제 해결)
            for j = 1:MPC_horizon+1
                MPC_pred2_history(i,:,j) = X_mpc2_pred(:,j);
            end
            
            % 첫 번째 제어 입력 적용
            mpc_control2_history(:,i) = u_opt2;
        else
            % 제어 간격이 아닌 경우 이전 값 유지
            mpc_control1_history(:,i) = mpc_control1_history(:,i-1);
            mpc_control2_history(:,i) = mpc_control2_history(:,i-1);
        end
        
        % 6. 실제 제어 입력 선택 (여기서는 PPID 제어기 사용)
        final_cmd = final_cmd_PPID;
        control_input_history(:,i) = final_cmd;
        
        % 7. 각 선형화 모델에 따른 1단계 예측
        X_lin1_next = Ad1 * X + Bd1 * final_cmd;
        X_lin2_next = Ad2_full * X + Bd2_full * final_cmd;
        
        % 쿼터니언 정규화
        X_lin1_next(7:10) = X_lin1_next(7:10) / norm(X_lin1_next(7:10));
        X_lin2_next(7:10) = X_lin2_next(7:10) / norm(X_lin2_next(7:10));
        
        % 8. 비선형 동역학 시뮬레이션 (SRK4)
        [X_next_nonlin, Thrust_mass, alpha_tot, phi_A, F_aero, M_aero, moment_coupling] = ...
            srk4(@vehicle_dynamics_HANul, X, final_cmd, Qf, dt_sim, params, current_time, dt_sim);
        
        % 9. 예측 오차 계산
        lin1_error = X_next_nonlin - X_lin1_next;
        lin2_error = X_next_nonlin - X_lin2_next;
        
        % MPC 예측 오차 계산 (첫 번째 예측 스텝에 대해)
        if mod(i-1, MPC_control_interval) == 0
            mpc1_error = X_next_nonlin - squeeze(MPC_pred1_history(i,:,2))';
            mpc2_error = X_next_nonlin - squeeze(MPC_pred2_history(i,:,2))';
        else
            mpc1_error = zeros(13, 1);
            mpc2_error = zeros(13, 1);
        end
        
        % 10. 결과 저장
        lin1_error_history(:,i+1) = lin1_error;
        lin2_error_history(:,i+1) = lin2_error;
        mpc1_error_history(:,i+1) = mpc1_error;
        mpc2_error_history(:,i+1) = mpc2_error;
        
        X_lin1_history(:,i+1) = X_lin1_next;
        X_lin2_history(:,i+1) = X_lin2_next;
        
        % 11. 상태 업데이트
        X = X_next_nonlin;
        state_history(:,i+1) = X;
        
        % 12. 진행 상태 표시
        if mod(i, update_interval) == 0 || i == num_steps-1
            progress = i / (num_steps-1);
            waitbar(progress, hWait, sprintf('진행 중: %.2f%%', progress * 100));
        end
    end
    
    % 마지막 상태 기록
    control_input_history(:,num_steps) = final_cmd;
    mpc_control1_history(:,num_steps) = mpc_control1_history(:,num_steps-1);
    mpc_control2_history(:,num_steps) = mpc_control2_history(:,num_steps-1);
    
    % 진행 상태 창 닫기
    close(hWait);
    
catch ME
    % 오류 발생 시 진행 상태 창 닫고 오류 출력
    close(hWait);
    rethrow(ME);
end

%% 결과 분석 및 시각화

% 1. 오차 통계 계산
lin1_error_norm = zeros(num_steps, 1);
lin2_error_norm = zeros(num_steps, 1);
mpc1_error_norm = zeros(num_steps, 1);
mpc2_error_norm = zeros(num_steps, 1);

lin1_pos_error_norm = zeros(num_steps, 1);
lin2_pos_error_norm = zeros(num_steps, 1);
lin1_vel_error_norm = zeros(num_steps, 1);
lin2_vel_error_norm = zeros(num_steps, 1);
lin1_quat_error_norm = zeros(num_steps, 1);
lin2_quat_error_norm = zeros(num_steps, 1);
lin1_omega_error_norm = zeros(num_steps, 1);
lin2_omega_error_norm = zeros(num_steps, 1);

for i = 1:num_steps
    % 전체 상태 오차 계산
    lin1_error_norm(i) = norm(lin1_error_history(:,i));
    lin2_error_norm(i) = norm(lin2_error_history(:,i));
    mpc1_error_norm(i) = norm(mpc1_error_history(:,i));
    mpc2_error_norm(i) = norm(mpc2_error_history(:,i));
    
    % 상태 구성요소별 오차 계산
    lin1_pos_error_norm(i) = norm(lin1_error_history(1:3,i));
    lin2_pos_error_norm(i) = norm(lin2_error_history(1:3,i));
    lin1_vel_error_norm(i) = norm(lin1_error_history(4:6,i));
    lin2_vel_error_norm(i) = norm(lin2_error_history(4:6,i));
    lin1_quat_error_norm(i) = norm(lin1_error_history(7:10,i));
    lin2_quat_error_norm(i) = norm(lin2_error_history(7:10,i));
    lin1_omega_error_norm(i) = norm(lin1_error_history(11:13,i));
    lin2_omega_error_norm(i) = norm(lin2_error_history(11:13,i));
end

% 2. 선형화 행렬 비교 그래프
figure(1);
subplot(2,1,1);
plot(t(1:end-1), matrix_norm_diff_history(1:end-1));
title('선형화 행렬 차이 (상대 Frobenius 노름)');
xlabel('시간 [s]'); ylabel('상대 차이');
grid on;

subplot(2,1,2);
plot(t(1:end-1), eigenvalue_diff_history(1:end-1));
title('선형화 행렬 고유값 최대 차이');
xlabel('시간 [s]'); ylabel('최대 고유값 차이');
grid on;

% 3. 위치 및 속도 예측 비교 그래프
figure(2);
subplot(3,2,1);
plot(t, state_history(1,:), 'k-', t, X_lin1_history(1,:), 'b--', t, X_lin2_history(1,:), 'r-.');
title('X 위치'); xlabel('시간 [s]'); ylabel('위치 [m]');
legend('비선형 (SRK4)', 'linearize\_rocket', 'linearize\_HANul\_quaternion');
grid on;

subplot(3,2,2);
plot(t, state_history(4,:), 'k-', t, X_lin1_history(4,:), 'b--', t, X_lin2_history(4,:), 'r-.');
title('u 속도'); xlabel('시간 [s]'); ylabel('속도 [m/s]');
legend('비선형 (SRK4)', 'linearize\_rocket', 'linearize\_HANul\_quaternion');
grid on;

subplot(3,2,3);
plot(t, state_history(2,:), 'k-', t, X_lin1_history(2,:), 'b--', t, X_lin2_history(2,:), 'r-.');
title('Y 위치'); xlabel('시간 [s]'); ylabel('위치 [m]');
legend('비선형 (SRK4)', 'linearize\_rocket', 'linearize\_HANul\_quaternion');
grid on;

subplot(3,2,4);
plot(t, state_history(5,:), 'k-', t, X_lin1_history(5,:), 'b--', t, X_lin2_history(5,:), 'r-.');
title('v 속도'); xlabel('시간 [s]'); ylabel('속도 [m/s]');
legend('비선형 (SRK4)', 'linearize\_rocket', 'linearize\_HANul\_quaternion');
grid on;

subplot(3,2,5);
plot(t, -state_history(3,:), 'k-', t, -X_lin1_history(3,:), 'b--', t, -X_lin2_history(3,:), 'r-.');
title('고도 (Z)'); xlabel('시간 [s]'); ylabel('고도 [m]');
legend('비선형 (SRK4)', 'linearize\_rocket', 'linearize\_HANul\_quaternion');
grid on;

subplot(3,2,6);
plot(t, state_history(6,:), 'k-', t, X_lin1_history(6,:), 'b--', t, X_lin2_history(6,:), 'r-.');
title('w 속도'); xlabel('시간 [s]'); ylabel('속도 [m/s]');
legend('비선형 (SRK4)', 'linearize\_rocket', 'linearize\_HANul\_quaternion');
grid on;

% 4. 각속도 예측 비교 그래프
figure(3);
subplot(3,1,1);
plot(t, state_history(11,:), 'k-', t, X_lin1_history(11,:), 'b--', t, X_lin2_history(11,:), 'r-.');
title('롤 각속도 (p)'); xlabel('시간 [s]'); ylabel('각속도 [rad/s]');
legend('비선형 (SRK4)', 'linearize\_rocket', 'linearize\_HANul\_quaternion');
grid on;

subplot(3,1,2);
plot(t, state_history(12,:), 'k-', t, X_lin1_history(12,:), 'b--', t, X_lin2_history(12,:), 'r-.');
title('피치 각속도 (q)'); xlabel('시간 [s]'); ylabel('각속도 [rad/s]');
legend('비선형 (SRK4)', 'linearize\_rocket', 'linearize\_HANul\_quaternion');
grid on;

subplot(3,1,3);
plot(t, state_history(13,:), 'k-', t, X_lin1_history(13,:), 'b--', t, X_lin2_history(13,:), 'r-.');
title('요 각속도 (r)'); xlabel('시간 [s]'); ylabel('각속도 [rad/s]');
legend('비선형 (SRK4)', 'linearize\_rocket', 'linearize\_HANul\_quaternion');
grid on;

% 5. 쿼터니언 예측 비교 그래프
figure(4);
subplot(4,1,1);
plot(t, state_history(7,:), 'k-', t, X_lin1_history(7,:), 'b--', t, X_lin2_history(7,:), 'r-.');
title('쿼터니언 q0'); xlabel('시간 [s]'); ylabel('q0');
legend('비선형 (SRK4)', 'linearize\_rocket', 'linearize\_HANul\_quaternion');
grid on;

subplot(4,1,2);
plot(t, state_history(8,:), 'k-', t, X_lin1_history(8,:), 'b--', t, X_lin2_history(8,:), 'r-.');
title('쿼터니언 q1'); xlabel('시간 [s]'); ylabel('q1');
legend('비선형 (SRK4)', 'linearize\_rocket', 'linearize\_HANul\_quaternion');
grid on;

subplot(4,1,3);
plot(t, state_history(9,:), 'k-', t, X_lin1_history(9,:), 'b--', t, X_lin2_history(9,:), 'r-.');
title('쿼터니언 q2'); xlabel('시간 [s]'); ylabel('q2');
legend('비선형 (SRK4)', 'linearize\_rocket', 'linearize\_HANul\_quaternion');
grid on;

subplot(4,1,4);
plot(t, state_history(10,:), 'k-', t, X_lin1_history(10,:), 'b--', t, X_lin2_history(10,:), 'r-.');
title('쿼터니언 q3'); xlabel('시간 [s]'); ylabel('q3');
legend('비선형 (SRK4)', 'linearize\_rocket', 'linearize\_HANul\_quaternion');
grid on;

% 6. 예측 오차 비교
figure(5);
subplot(3,1,1);
semilogy(t, lin1_error_norm, 'b-', t, lin2_error_norm, 'r-');
title('선형화 모델 예측 오차 (로그 스케일)');
xlabel('시간 [s]'); ylabel('오차 노름');
legend('linearize\_rocket', 'linearize\_HANul\_quaternion');
grid on;

subplot(3,1,2);
semilogy(t, lin1_quat_error_norm, 'b-', t, lin2_quat_error_norm, 'r-');
title('쿼터니언 예측 오차');
xlabel('시간 [s]'); ylabel('오차 노름');
legend('linearize\_rocket', 'linearize\_HANul\_quaternion');
grid on;

subplot(3,1,3);
semilogy(t, lin1_omega_error_norm, 'b-', t, lin2_omega_error_norm, 'r-');
title('각속도 예측 오차');
xlabel('시간 [s]'); ylabel('오차 노름');
legend('linearize\_rocket', 'linearize\_HANul\_quaternion');
grid on;

% 7. MPC 예측 궤적 시각화 (선택된 시간에서)
% MPC 예측 궤적을 보기 위한 시간 인덱스 선택
selected_time_idx = 100;  % 적절한 시간 인덱스 선택
if selected_time_idx <= num_steps
    % MPC 예측 시간 벡터
    t_mpc = (0:MPC_horizon) * MPC_dt;
    
    % 예측 궤적 추출 (수정된 방식)
    mpc1_traj = zeros(MPC_horizon+1, 13);
    mpc2_traj = zeros(MPC_horizon+1, 13);
    
    for j = 1:MPC_horizon+1
        mpc1_traj(j,:) = squeeze(MPC_pred1_history(selected_time_idx,:,j));
        mpc2_traj(j,:) = squeeze(MPC_pred2_history(selected_time_idx,:,j));
    end
    
    % 실제 궤적 인덱스 계산
    actual_idx_range = selected_time_idx:min(selected_time_idx+MPC_horizon, num_steps);
    
    % 궤적 시각화
    figure(6);
    
    % 롤 각속도 예측
    subplot(3,1,1);
    plot(t_mpc, mpc1_traj(:,11), 'b-', t_mpc, mpc2_traj(:,11), 'r-', ...
        t(actual_idx_range) - t(selected_time_idx), state_history(11,actual_idx_range), 'k--');
    title('롤 각속도 (p) MPC 예측'); xlabel('예측 시간 [s]'); ylabel('각속도 [rad/s]');
    legend('linearize\_rocket MPC', 'linearize\_HANul\_quaternion MPC', '실제 궤적');
    grid on;
    
    % 피치 각속도 예측
    subplot(3,1,2);
    plot(t_mpc, mpc1_traj(:,12), 'b-', t_mpc, mpc2_traj(:,12), 'r-', ...
        t(actual_idx_range) - t(selected_time_idx), state_history(12,actual_idx_range), 'k--');
    title('피치 각속도 (q) MPC 예측'); xlabel('예측 시간 [s]'); ylabel('각속도 [rad/s]');
    legend('linearize\_rocket MPC', 'linearize\_HANul\_quaternion MPC', '실제 궤적');
    grid on;
    
    % 요 각속도 예측
    subplot(3,1,3);
    plot(t_mpc, mpc1_traj(:,13), 'b-', t_mpc, mpc2_traj(:,13), 'r-', ...
        t(actual_idx_range) - t(selected_time_idx), state_history(13,actual_idx_range), 'k--');
    title('요 각속도 (r) MPC 예측'); xlabel('예측 시간 [s]'); ylabel('각속도 [rad/s]');
    legend('linearize\_rocket MPC', 'linearize\_HANul\_quaternion MPC', '실제 궤적');
    grid on;
    
    % 쿼터니언 예측
    figure(7);
    subplot(4,1,1);
    plot(t_mpc, mpc1_traj(:,7), 'b-', t_mpc, mpc2_traj(:,7), 'r-', ...
        t(actual_idx_range) - t(selected_time_idx), state_history(7,actual_idx_range), 'k--');
    title('쿼터니언 q0 MPC 예측'); xlabel('예측 시간 [s]'); ylabel('q0');
    legend('linearize\_rocket MPC', 'linearize\_HANul\_quaternion MPC', '실제 궤적');
    grid on;
    
    subplot(4,1,2);
    plot(t_mpc, mpc1_traj(:,8), 'b-', t_mpc, mpc2_traj(:,8), 'r-', ...
        t(actual_idx_range) - t(selected_time_idx), state_history(8,actual_idx_range), 'k--');
    title('쿼터니언 q1 MPC 예측'); xlabel('예측 시간 [s]'); ylabel('q1');
    legend('linearize\_rocket MPC', 'linearize\_HANul\_quaternion MPC', '실제 궤적');
    grid on;
    
    subplot(4,1,3);
    plot(t_mpc, mpc1_traj(:,9), 'b-', t_mpc, mpc2_traj(:,9), 'r-', ...
        t(actual_idx_range) - t(selected_time_idx), state_history(9,actual_idx_range), 'k--');
    title('쿼터니언 q2 MPC 예측'); xlabel('예측 시간 [s]'); ylabel('q2');
    legend('linearize\_rocket MPC', 'linearize\_HANul\_quaternion MPC', '실제 궤적');
    grid on;
    
    subplot(4,1,4);
    plot(t_mpc, mpc1_traj(:,10), 'b-', t_mpc, mpc2_traj(:,10), 'r-', ...
        t(actual_idx_range) - t(selected_time_idx), state_history(10,actual_idx_range), 'k--');
    title('쿼터니언 q3 MPC 예측'); xlabel('예측 시간 [s]'); ylabel('q3');
    legend('linearize\_rocket MPC', 'linearize\_HANul\_quaternion MPC', '실제 궤적');
    grid on;
end

% 8. 통계 정보 출력
fprintf('\n===== 선형화 모델 예측 오차 통계 =====\n');
fprintf('linearize_rocket 평균 오차: %f\n', mean(lin1_error_norm));
fprintf('linearize_HANul_quaternion 평균 오차: %f\n', mean(lin2_error_norm));
fprintf('linearize_rocket 쿼터니언 평균 오차: %f\n', mean(lin1_quat_error_norm));
fprintf('linearize_HANul_quaternion 쿼터니언 평균 오차: %f\n', mean(lin2_quat_error_norm));
fprintf('linearize_rocket 각속도 평균 오차: %f rad/s\n', mean(lin1_omega_error_norm));
fprintf('linearize_HANul_quaternion 각속도 평균 오차: %f rad/s\n', mean(lin2_omega_error_norm));

fprintf('\n===== 행렬 차이 통계 =====\n');
fprintf('평균 행렬 상대 노름 차이: %f\n', mean(matrix_norm_diff_history));
fprintf('평균 고유값 최대 차이: %f\n', mean(eigenvalue_diff_history));