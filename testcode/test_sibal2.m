%% HANul 로켓 선형화 시뮬레이션
% Last update: 2025-05-07

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
% 비선형 RK4 상태 기록
X_nonlin_history = zeros(length(X), num_steps);
X_nonlin_history(:,1) = X;

% 수치적 선형화 상태 기록
X_num_lin_history = zeros(length(X), num_steps);
X_num_lin_history(:,1) = X;

% 해석적 선형화 상태 기록
X_ana_lin_history = zeros(length(X), num_steps);
X_ana_lin_history(:,1) = X;

% 제어 및 추가 변수 기록
nonlin_control_history = zeros(9, num_steps);  % 비선형 RK4 제어 명령
num_lin_control_history = zeros(9, num_steps); % 수치적 선형화 제어 명령
ana_lin_control_history = zeros(9, num_steps); % 해석적 선형화 제어 명령

% 기타 기록 변수
Thrust_mass_history = zeros(4, num_steps);
alpha_tot_history = zeros(1, num_steps);
phi_A_history = zeros(1, num_steps);
F_aero_history = zeros(3, num_steps);
M_aero_history = zeros(3, num_steps);
moment_coupling_history = zeros(3, num_steps);

deltaU_ana = zeros(9,1);

% 선형화 관련 기록
num_lin_error_history = zeros(length(X), num_steps);
ana_lin_error_history = zeros(length(X), num_steps);
Ad_num_history = zeros(7, 7, num_steps);
Bd_num_history = zeros(7, 9, num_steps);
Ad_ana_history = zeros(13, 13, num_steps);
Bd_ana_history = zeros(13, 9, num_steps);

% 오일러 각 저장용 변수 초기화
euler_nonlinear = zeros(3, num_steps);
euler_num_linear = zeros(3, num_steps);
euler_ana_linear = zeros(3, num_steps);

% 제어 간격 설정
attitude_angle_interval = Sim_Loop_Hz / attitude_Control_Hz;

% ===== 그래프 초기화 =====
% 1. 위치 및 속도 그래프
figure(1);

subplot(3,2,1);
hold on;
title('X 위치'); xlabel('시간 [s]'); ylabel('위치 [m]');
grid on;

subplot(3,2,2);
hold on;
title('u 속도'); xlabel('시간 [s]'); ylabel('속도 [m/s]');
grid on;

subplot(3,2,3);
hold on;
title('Y 위치'); xlabel('시간 [s]'); ylabel('위치 [m]');
grid on;

subplot(3,2,4);
hold on;
title('v 속도'); xlabel('시간 [s]'); ylabel('속도 [m/s]');
grid on;

subplot(3,2,5);
hold on;
title('고도 (Z)'); xlabel('시간 [s]'); ylabel('고도 [m]');
grid on;

subplot(3,2,6);
hold on;
title('w 속도'); xlabel('시간 [s]'); ylabel('속도 [m/s]');
grid on;

% 2. 각속도 그래프
figure(2);

subplot(3,1,1);
hold on;
title('롤 각속도 (p)'); xlabel('시간 [s]'); ylabel('각속도 [rad/s]');
grid on;

subplot(3,1,2);
hold on;
title('피치 각속도 (q)'); xlabel('시간 [s]'); ylabel('각속도 [rad/s]');
grid on;

subplot(3,1,3);
hold on;
title('요 각속도 (r)'); xlabel('시간 [s]'); ylabel('각속도 [rad/s]');
grid on;

% 3. 쿼터니언 그래프
figure(3);

subplot(4,1,1);
hold on;
title('쿼터니언 q0'); xlabel('시간 [s]'); ylabel('q0');
grid on;

subplot(4,1,2);
hold on;
title('쿼터니언 q1'); xlabel('시간 [s]'); ylabel('q1');
grid on;

subplot(4,1,3);
hold on;
title('쿼터니언 q2'); xlabel('시간 [s]'); ylabel('q2');
grid on;

subplot(4,1,4);
hold on;
title('쿼터니언 q3'); xlabel('시간 [s]'); ylabel('q3');
grid on;

% 4. 오일러 각 그래프
figure(4);

subplot(3,1,1);
hold on;
title('롤 각도'); xlabel('시간 [s]'); ylabel('각도 [deg]');
grid on;

subplot(3,1,2);
hold on;
title('피치 각도'); xlabel('시간 [s]'); ylabel('각도 [deg]');
grid on;

subplot(3,1,3);
hold on;
title('요 각도'); xlabel('시간 [s]'); ylabel('각도 [deg]');
grid on;

% 5. 예측 오차 비교 그래프
figure(5);

subplot(5,1,1);
hold on;
title('전체 상태 벡터 예측 오차'); xlabel('시간 [s]'); ylabel('오차 (로그 스케일)');
grid on;

subplot(5,1,2);
hold on;
title('위치 예측 오차'); xlabel('시간 [s]'); ylabel('오차 [m]');
grid on;

subplot(5,1,3);
hold on;
title('속도 예측 오차'); xlabel('시간 [s]'); ylabel('오차 [m/s]');
grid on;

subplot(5,1,4);
hold on;
title('쿼터니언 예측 오차'); xlabel('시간 [s]'); ylabel('오차');
grid on;

subplot(5,1,5);
hold on;
title('각속도 예측 오차'); xlabel('시간 [s]'); ylabel('오차 [rad/s]');
grid on;

% 6. 수치적 vs 해석적 방법 오차 비교 그래프
figure(6);

subplot(2,1,1);
hold on;
title('쿼터니언 오차 차이 (수치적 - 해석적)'); 
xlabel('시간 [s]'); ylabel('오차 차이');
grid on;

subplot(2,1,2);
hold on;
title('각속도 오차 차이 (수치적 - 해석적)'); 
xlabel('시간 [s]'); ylabel('오차 차이 [rad/s]');
grid on;

% % ===== 순차적으로 시뮬레이션 실행 =====
% % 1. 비선형 RK4 시뮬레이션 루프
% fprintf('비선형 RK4 시뮬레이션 시작...\n');
% hWait = waitbar(0, '비선형 RK4 시뮬레이션 진행 중...', 'Name', '진행 상태');
update_interval = 100;
% 
% X_nonlin = X;
% try
%     for i = 1:num_steps-1
%         % 1.1 현재 시간 및 파라미터 업데이트
%         current_time = (i-1) * dt_sim;
%         params = vehicle_params(flatRocketParams, current_time, motorCsvPath, false);
% 
%         % 1.2 제어 명령 생성 및 할당
%         Command_Vector = generate_commands_HANul(i, 1/dt_sim);
% 
%         % 제어기 주기적 실행
%         if mod(i-1, attitude_angle_interval) == 0
%             U = zeros(6,1);  % U(1) 추력 추가를 위해 6x1로 초기화
%             U(4:6) = Controller_PPID_Argument(X_nonlin, Command_Vector(2:5), dt_sim, params);
%             U(1) = Command_Vector(1);  % 추력 명령
%         end 
% 
%         % 제어 명령 할당
%         [final_cmd, debug_cmd] = Control_Allocator_RCS(U, params);
% 
%         % 1.3 비선형 동역학 적분 (SRK4)
%         [X_nonlin_next, Thrust_mass, alpha_tot, phi_A, F_aero, M_aero, moment_coupling] = ...
%             srk4(@vehicle_dynamics_HANul, X_nonlin, final_cmd, Qf, dt_sim, params, current_time, dt_sim);
% 
%         % 1.4 상태 업데이트 및 데이터 기록
%         X_nonlin = X_nonlin_next;
%         X_nonlin_history(:,i+1) = X_nonlin;
%         nonlin_control_history(:,i) = final_cmd;
%         Thrust_mass_history(:,i) = Thrust_mass;
%         alpha_tot_history(i) = alpha_tot;
%         phi_A_history(i) = phi_A;
%         F_aero_history(:,i) = F_aero;
%         M_aero_history(:,i) = M_aero;
%         moment_coupling_history(:,i) = moment_coupling;
% 
%         % 1.5 오일러 각 계산
%         quat_nonlin = X_nonlin_history(7:10,i+1);
%         euler_nonlinear(:,i+1) = Quat2Euler(quat_nonlin);
% 
%         % 1.6 진행 상태 표시
%         if mod(i, update_interval) == 0 || i == num_steps-1
%             progress = i / (num_steps-1);
%             waitbar(progress, hWait, sprintf('비선형 RK4: %.2f%%', progress * 100));
%         end
%     end
% 
%     % 마지막 상태 기록
%     nonlin_control_history(:,num_steps) = final_cmd;
% 
%     % 진행 상태 창 닫기
%     close(hWait);
% 
% catch ME
%     % 오류 발생 시 진행 상태 창 닫고 오류 출력
%     close(hWait);
%     rethrow(ME);
% end
% 
% % 비선형 RK4 결과 그래프 업데이트
% fprintf('비선형 RK4 결과 그래프 작성 중...\n');
% 
% % 1. 위치 및 속도 그래프
% figure(1);
% subplot(3,2,1);
% plot(t, X_nonlin_history(1,:), 'b-');
% legend('비선형 (SRK4)');
% 
% subplot(3,2,2);
% plot(t, X_nonlin_history(4,:), 'b-');
% legend('비선형 (SRK4)');
% 
% subplot(3,2,3);
% plot(t, X_nonlin_history(2,:), 'b-');
% legend('비선형 (SRK4)');
% 
% subplot(3,2,4);
% plot(t, X_nonlin_history(5,:), 'b-');
% legend('비선형 (SRK4)');
% 
% subplot(3,2,5);
% plot(t, -X_nonlin_history(3,:), 'b-');
% legend('비선형 (SRK4)');
% 
% subplot(3,2,6);
% plot(t, X_nonlin_history(6,:), 'b-');
% legend('비선형 (SRK4)');
% 
% % 2. 각속도 그래프
% figure(2);
% subplot(3,1,1);
% plot(t, X_nonlin_history(11,:), 'b-');
% legend('비선형 (SRK4)');
% 
% subplot(3,1,2);
% plot(t, X_nonlin_history(12,:), 'b-');
% legend('비선형 (SRK4)');
% 
% subplot(3,1,3);
% plot(t, X_nonlin_history(13,:), 'b-');
% legend('비선형 (SRK4)');
% 
% % 3. 쿼터니언 그래프
% figure(3);
% subplot(4,1,1);
% plot(t, X_nonlin_history(7,:), 'b-');
% legend('비선형 (SRK4)');
% 
% subplot(4,1,2);
% plot(t, X_nonlin_history(8,:), 'b-');
% legend('비선형 (SRK4)');
% 
% subplot(4,1,3);
% plot(t, X_nonlin_history(9,:), 'b-');
% legend('비선형 (SRK4)');
% 
% subplot(4,1,4);
% plot(t, X_nonlin_history(10,:), 'b-');
% legend('비선형 (SRK4)');
% 
% % 4. 오일러 각 그래프
% figure(4);
% subplot(3,1,1);
% plot(t, rad2deg(euler_nonlinear(1,:)), 'b-');
% legend('비선형 (SRK4)');
% 
% subplot(3,1,2);
% plot(t, rad2deg(euler_nonlinear(2,:)), 'b-');
% legend('비선형 (SRK4)');
% 
% subplot(3,1,3);
% plot(t, rad2deg(euler_nonlinear(3,:)), 'b-');
% legend('비선형 (SRK4)');

% % ===== 2. 수치적 선형화 시뮬레이션 루프 (자세와 각속도만) =====
% fprintf('수치적 선형화 시뮬레이션 시작... (자세와 각속도만)\n');
% hWait = waitbar(0, '수치적 선형화 시뮬레이션 진행 중...', 'Name', '진행 상태');
% 
% % 수치적 선형화를 위한 자세/각속도 벡터 초기화
% X_num_lin = X;
% X_att = X(7:13);      % 자세(쿼터니언) + 각속도
% X_att_prev = X_att;
% final_cmd_prev = zeros(9, 1);  % 9x1로 수정 (추력 제어 추가)
% 
% try
%     for i = 1:num_steps-1
%         % 2.1 현재 시간 및 파라미터 업데이트
%         current_time = (i-1) * dt_sim;
%         params = vehicle_params(flatRocketParams, current_time, motorCsvPath, false);
% 
%         % 2.2 제어 명령 생성 및 할당
%         Command_Vector = generate_commands_HANul(i, 1/dt_sim);
% 
%         % 제어기 주기적 실행
%         if mod(i-1, attitude_angle_interval) == 0
%             U = zeros(6,1);  % U(1) 추력 추가를 위해 6x1로 초기화
%             U(4:6) = Controller_PPID_Argument(X_num_lin, Command_Vector(2:5), dt_sim, params);
%             U(1) = Command_Vector(1);  % 추력 명령
%         end 
% 
%         % 제어 명령 할당
%         [final_cmd, debug_cmd] = Control_Allocator_RCS(U, params);
% 
%         % 2.3 수치적 선형화 수행 (자세와 각속도만)
%         [Ad_num, Bd_num] = linearize_HANul_quaternion(X_num_lin, final_cmd, params, dt_sim);
% 
%         % 선형화 결과 저장
%         Ad_num_history(:,:,i) = Ad_num;
%         Bd_num_history(:,:,i) = Bd_num;
% 
%         % 2.4 선형화 모델을 사용한 다음 상태 예측
%         X_num_lin_next = X_num_lin;
% 
%         % 비선형 RK4로 위치와 속도 계산 
%         [X_nonlin_next, ~, ~, ~, ~, ~, ~] = ...
%             srk4(@vehicle_dynamics_HANul, X_num_lin, final_cmd, Qf, dt_sim, params, current_time, dt_sim);
% 
%         % 위치와 속도는 비선형 결과 사용
%         X_num_lin_next(1:6) = X_nonlin_next(1:6);
% 
%         % 자세와 각속도는 선형화 모델로 예측
%         X_att = X_num_lin(7:13);
%         X_att_next = X_att_prev + Ad_num * (X_att - X_att_prev) + Bd_num * final_cmd;%(final_cmd - final_cmd_prev);
%         X_num_lin_next(7:13) = X_att_next;
% 
%         X_num_lin_history(:,i+1) = X_num_lin_next;
%         num_lin_control_history(:,i) = final_cmd;
% 
%         % 2.5 상태 및 입력 업데이트
%         X_att_prev = X_att;
%         X_num_lin = X_num_lin_next;
%         final_cmd_prev = final_cmd;
% 
%         % 2.6 선형화 예측과 비선형 결과 비교
%         num_lin_error = X_nonlin_history(:,i+1) - X_num_lin_next;
%         num_lin_error_history(:,i+1) = num_lin_error;
% 
%         % 2.7 오일러 각 계산
%         quat_num_lin = X_num_lin_history(7:10,i+1);
%         euler_num_linear(:,i+1) = Quat2Euler(quat_num_lin);
% 
%         % 2.8 진행 상태 표시
%         if mod(i, update_interval) == 0 || i == num_steps-1
%             progress = i / (num_steps-1);
%             waitbar(progress, hWait, sprintf('수치적 선형화: %.2f%%', progress * 100));
%         end
%     end
% 
%     % 마지막 상태 기록
%     num_lin_control_history(:,num_steps) = final_cmd;
% 
%     % 진행 상태 창 닫기
%     close(hWait);
% 
% catch ME
%     % 오류 발생 시 진행 상태 창 닫고 오류 출력
%     close(hWait);
%     rethrow(ME);
% end
% 
% % 수치적 선형화 결과 그래프 업데이트
% fprintf('수치적 선형화 결과 그래프 추가 중...\n');
% 
% % 각속도 그래프에 추가
% figure(2);
% subplot(3,1,1);
% plot(t, X_num_lin_history(11,:), 'r--');
% legend('비선형 (SRK4)', '수치적 선형화');
% 
% subplot(3,1,2);
% plot(t, X_num_lin_history(12,:), 'r--');
% legend('비선형 (SRK4)', '수치적 선형화');
% 
% subplot(3,1,3);
% plot(t, X_num_lin_history(13,:), 'r--');
% legend('비선형 (SRK4)', '수치적 선형화');
% 
% % 쿼터니언 그래프에 추가
% figure(3);
% subplot(4,1,1);
% plot(t, X_num_lin_history(7,:), 'r--');
% legend('비선형 (SRK4)', '수치적 선형화');
% 
% subplot(4,1,2);
% plot(t, X_num_lin_history(8,:), 'r--');
% legend('비선형 (SRK4)', '수치적 선형화');
% 
% subplot(4,1,3);
% plot(t, X_num_lin_history(9,:), 'r--');
% legend('비선형 (SRK4)', '수치적 선형화');
% 
% subplot(4,1,4);
% plot(t, X_num_lin_history(10,:), 'r--');
% legend('비선형 (SRK4)', '수치적 선형화');
% 
% % 오일러 각 그래프에 추가
% figure(4);
% subplot(3,1,1);
% plot(t, rad2deg(euler_num_linear(1,:)), 'r--');
% legend('비선형 (SRK4)', '수치적 선형화');
% 
% subplot(3,1,2);
% plot(t, rad2deg(euler_num_linear(2,:)), 'r--');
% legend('비선형 (SRK4)', '수치적 선형화');
% 
% subplot(3,1,3);
% plot(t, rad2deg(euler_num_linear(3,:)), 'r--');
% legend('비선형 (SRK4)', '수치적 선형화');
% 
% % 예측 오차 그래프 (수치적 선형화) - 자세와 각속도 오차만 표시
% figure(5);
% subplot(5,1,4);
% semilogy(t, arrayfun(@(i) norm(num_lin_error_history(7:10,i)), 1:length(t)), 'r-');
% legend('수치적 선형화 오차 (쿼터니언)');
% 
% subplot(5,1,5);
% semilogy(t, arrayfun(@(i) norm(num_lin_error_history(11:13,i)), 1:length(t)), 'r-');
% legend('수치적 선형화 오차 (각속도)');

% ===== 3. 해석적 선형화 시뮬레이션 루프 =====
fprintf('해석적 선형화 시뮬레이션 시작...\n');
hWait = waitbar(0, '해석적 선형화 시뮬레이션 진행 중...', 'Name', '진행 상태');
X_ana_lin = X;
X_ana_lin_prev = X;
final_cmd_prev = zeros(9, 1);  % 9x1로 수정 (추력 제어 추가)

try
    for i = 1:num_steps-1
        % 3.1 현재 시간 및 파라미터 업데이트
        current_time = (i-1) * dt_sim;
        params = vehicle_params(flatRocketParams, current_time, motorCsvPath, false);
        
        % 3.2 제어 명령 생성 및 할당
        Command_Vector = generate_commands_HANul(i, 1/dt_sim);
        
        % 제어기 주기적 실행
        if mod(i-1, attitude_angle_interval) == 0
            U = zeros(6,1);  % U(1) 추력 추가를 위해 6x1로 초기화
            U(4:6) = Controller_PPID_Argument(X_ana_lin, Command_Vector(2:5), dt_sim, params);
            U(1) = Command_Vector(1);  % 추력 명령
        end 
        
        % 제어 명령 할당
        [final_cmd, debug_cmd] = Control_Allocator_RCS(U, params);
        
        % 3.3 해석적 선형화 수행
        [Ad_ana, Bd_ana] = linearize_rocket(X_ana_lin, final_cmd, params, dt_sim);
        
        % 선형화 결과 저장
        Ad_ana_history(:,:,i) = Ad_ana;
        Bd_ana_history(:,:,i) = Bd_ana;
        
        
        % 3.4 선형화 모델을 사용한 다음 상태 예측
        X_ana_lin_next = Ad_ana * X_ana_lin + Bd_ana * final_cmd;
        disp('X_ana_lin_next')
        disp(X_ana_lin_next)

        % disp('Ad_ana')
        % disp(Ad_ana)
        % 
        disp('Bd_ana * final_cmd')
        disp(Bd_ana * final_cmd)

        if norm(X_ana_lin_next(7:10)) > 1e-9 % norm이 0에 매우 가까우면 나누기 오류 방지
            X_ana_lin_next(7:10) = X_ana_lin_next(7:10) / norm(X_ana_lin_next(7:10));
        else
            % 쿼터니언 norm이 0에 가까운 경우, 초기값 등으로 리셋하거나 오류 처리
            warning('Quaternion norm is close to zero at step %d', i);
            % 예: X_ana_lin_next(7:10) = [1; 0; 0; 0]; % 또는 다른 적절한 값
        end
        
        % disp('X_ana_lin_next after normalization') % 디버깅용
        % disp(X_ana_lin_next)

        X_ana_lin_history(:,i+1) = X_ana_lin_next;
        ana_lin_control_history(:,i) = final_cmd;

        X_ana_lin_history(:,i+1) = X_ana_lin_next;
        ana_lin_control_history(:,i) = final_cmd;
               
        % 3.5 상태 및 입력 업데이트        
        X_ana_lin_prev = X_ana_lin;
        X_ana_lin = X_ana_lin_next;
        final_cmd_prev = final_cmd;

        disp('step')
        disp(i)
        disp('current_time')
        disp(current_time)
        
        % 3.6 선형화 예측과 비선형 결과 비교
        ana_lin_error = X_nonlin_history(:,i+1) - X_ana_lin_next;
        ana_lin_error_history(:,i+1) = ana_lin_error;
        
        % 3.7 오일러 각 계산
        quat_ana_lin = X_ana_lin_history(7:10,i+1);
        euler_ana_linear(:,i+1) = Quat2Euler(quat_ana_lin);
        
        % 3.8 진행 상태 표시
        if mod(i, update_interval) == 0 || i == num_steps-1
            progress = i / (num_steps-1);
            waitbar(progress, hWait, sprintf('해석적 선형화: %.2f%%', progress * 100));
        end
    end
    
    % 마지막 상태 기록
    ana_lin_control_history(:,num_steps) = final_cmd;
    
    % 진행 상태 창 닫기
    close(hWait);
    
catch ME
    % 오류 발생 시 진행 상태 창 닫고 오류 출력
    close(hWait);
    rethrow(ME);
end

% 해석적 선형화 결과 그래프 업데이트
fprintf('해석적 선형화 결과 그래프 추가 중...\n');

% 1. 위치 및 속도 그래프
figure(1);
subplot(3,2,1);
plot(t, X_ana_lin_history(1,:), 'g-.');
legend('비선형 (SRK4)', '해석적 선형화');

subplot(3,2,2);
plot(t, X_ana_lin_history(4,:), 'g-.');
legend('비선형 (SRK4)', '해석적 선형화');

subplot(3,2,3);
plot(t, X_ana_lin_history(2,:), 'g-.');
legend('비선형 (SRK4)', '해석적 선형화');

subplot(3,2,4);
plot(t, X_ana_lin_history(5,:), 'g-.');
legend('비선형 (SRK4)', '해석적 선형화');

subplot(3,2,5);
plot(t, -X_ana_lin_history(3,:), 'g-.');
legend('비선형 (SRK4)', '해석적 선형화');

subplot(3,2,6);
plot(t, X_ana_lin_history(6,:), 'g-.');
legend('비선형 (SRK4)', '해석적 선형화');

% 2. 각속도 그래프
figure(2);
subplot(3,1,1);
plot(t, X_ana_lin_history(11,:), 'g-.');
legend('비선형 (SRK4)', '수치적 선형화', '해석적 선형화');

subplot(3,1,2);
plot(t, X_ana_lin_history(12,:), 'g-.');
legend('비선형 (SRK4)', '수치적 선형화', '해석적 선형화');

subplot(3,1,3);
plot(t, X_ana_lin_history(13,:), 'g-.');
legend('비선형 (SRK4)', '수치적 선형화', '해석적 선형화');

% 3. 쿼터니언 그래프
figure(3);
subplot(4,1,1);
plot(t, X_ana_lin_history(7,:), 'g-.');
legend('비선형 (SRK4)', '수치적 선형화', '해석적 선형화');

subplot(4,1,2);
plot(t, X_ana_lin_history(8,:), 'g-.');
legend('비선형 (SRK4)', '수치적 선형화', '해석적 선형화');

subplot(4,1,3);
plot(t, X_ana_lin_history(9,:), 'g-.');
legend('비선형 (SRK4)', '수치적 선형화', '해석적 선형화');

subplot(4,1,4);
plot(t, X_ana_lin_history(10,:), 'g-.');
legend('비선형 (SRK4)', '수치적 선형화', '해석적 선형화');

% 4. 오일러 각 그래프
figure(4);
subplot(3,1,1);
plot(t, rad2deg(euler_ana_linear(1,:)), 'g-.');
legend('비선형 (SRK4)', '수치적 선형화', '해석적 선형화');

subplot(3,1,2);
plot(t, rad2deg(euler_ana_linear(2,:)), 'g-.');
legend('비선형 (SRK4)', '수치적 선형화', '해석적 선형화');

subplot(3,1,3);
plot(t, rad2deg(euler_ana_linear(3,:)), 'g-.');
legend('비선형 (SRK4)', '수치적 선형화', '해석적 선형화');

% 5. 예측 오차 그래프 (해석적 선형화 추가)
figure(5);
subplot(5,1,1);
semilogy(t, arrayfun(@(i) norm(ana_lin_error_history(:,i)), 1:length(t)), 'g-');
legend('해석적 선형화 오차 (전체)');

subplot(5,1,2);
semilogy(t, arrayfun(@(i) norm(ana_lin_error_history(1:3,i)), 1:length(t)), 'g-');
legend('해석적 선형화 오차 (위치)');

subplot(5,1,3);
semilogy(t, arrayfun(@(i) norm(ana_lin_error_history(4:6,i)), 1:length(t)), 'g-');
legend('해석적 선형화 오차 (속도)');

subplot(5,1,4);
semilogy(t, arrayfun(@(i) norm(ana_lin_error_history(7:10,i)), 1:length(t)), 'g-');
legend('수치적 선형화 오차 (쿼터니언)', '해석적 선형화 오차 (쿼터니언)');

subplot(5,1,5);
semilogy(t, arrayfun(@(i) norm(ana_lin_error_history(11:13,i)), 1:length(t)), 'g-');
legend('수치적 선형화 오차 (각속도)', '해석적 선형화 오차 (각속도)');

% 6. 수치적 vs 해석적 방법 오차 비교
figure(6);
subplot(2,1,1);
plot(t, arrayfun(@(i) norm(num_lin_error_history(7:10,i)), 1:length(t)) - arrayfun(@(i) norm(ana_lin_error_history(7:10,i)), 1:length(t)));
title('쿼터니언 오차 차이 (수치적 - 해석적)'); 
xlabel('시간 [s]'); ylabel('오차 차이');
grid on;

subplot(2,1,2);
plot(t, arrayfun(@(i) norm(num_lin_error_history(11:13,i)), 1:length(t)) - arrayfun(@(i) norm(ana_lin_error_history(11:13,i)), 1:length(t)));
title('각속도 오차 차이 (수치적 - 해석적)'); 
xlabel('시간 [s]'); ylabel('오차 차이 [rad/s]');
grid on;

% ===== 오차 통계 계산 및 출력 =====
% 수치적 선형화 오차 통계
num_lin_error_quat_norm = arrayfun(@(i) norm(num_lin_error_history(7:10,i)), 1:num_steps);
num_lin_error_omega_norm = arrayfun(@(i) norm(num_lin_error_history(11:13,i)), 1:num_steps);

% 해석적 선형화 오차 통계
ana_lin_error_norm = arrayfun(@(i) norm(ana_lin_error_history(:,i)), 1:num_steps);
ana_lin_error_pos_norm = arrayfun(@(i) norm(ana_lin_error_history(1:3,i)), 1:num_steps);
ana_lin_error_vel_norm = arrayfun(@(i) norm(ana_lin_error_history(4:6,i)), 1:num_steps);
ana_lin_error_quat_norm = arrayfun(@(i) norm(ana_lin_error_history(7:10,i)), 1:num_steps);
ana_lin_error_omega_norm = arrayfun(@(i) norm(ana_lin_error_history(11:13,i)), 1:num_steps);

fprintf('\n===== 시뮬레이션 완료 =====\n');
fprintf('\n===== 수치적 선형화 모델 예측 오차 통계 (자세/각속도만 선형화) =====\n');
fprintf('쿼터니언 평균 오차: %f\n', mean(num_lin_error_quat_norm));
fprintf('각속도 평균 오차: %f rad/s\n', mean(num_lin_error_omega_norm));

fprintf('\n===== 해석적 선형화 모델 예측 오차 통계 =====\n');
fprintf('전체 상태 벡터 평균 오차: %f\n', mean(ana_lin_error_norm));
fprintf('위치 평균 오차: %f m\n', mean(ana_lin_error_pos_norm));
fprintf('속도 평균 오차: %f m/s\n', mean(ana_lin_error_vel_norm));
fprintf('쿼터니언 평균 오차: %f\n', mean(ana_lin_error_quat_norm));
fprintf('각속도 평균 오차: %f rad/s\n', mean(ana_lin_error_omega_norm));

fprintf('\n===== 선형화 방법 간 오차 차이 통계 (수치적 - 해석적) =====\n');
fprintf('쿼터니언 평균 오차 차이: %f\n', mean(num_lin_error_quat_norm - ana_lin_error_quat_norm));
fprintf('각속도 평균 오차 차이: %f rad/s\n', mean(num_lin_error_omega_norm - ana_lin_error_omega_norm));