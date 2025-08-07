clear all;
close all;
clc;
addpath(genpath('C:\Users\USER\Desktop\hanul_GNC'));

% ===== 로켓 파라미터 로딩 =====
fprintf('로켓 파라미터 로드 및 전처리 시작...\n');
% XML → 파라미터 트리
xmlFilePath = 'rocket.xml';
rocketParams = extractRocketParams(xmlFilePath);
flatRocketParams = flattenRocketStructure(rocketParams);

% ===== 모터 데이터 로드 =====
motorCsvPath = 'AeroTech_M2400T.csv';
opts = detectImportOptions(motorCsvPath, 'VariableNamingRule', 'preserve');
motorData = readtable(motorCsvPath, opts);
timeArray = motorData.("Time(s)");
thrustArray = motorData.("Thrust(N)");
simTime = max(timeArray);
dt = 1/400;

% ===== 결과 저장 배열 초기화 =====
time_steps = 0:dt:simTime;
num_steps = length(time_steps);
mass_history = zeros(num_steps, 1);
cg_abs_history = zeros(num_steps, 3);  % 절대 좌표계에서의 CG 위치
cg_delta_history = zeros(num_steps, 3); % CG 변화량
ixx_history = zeros(num_steps, 1);
iyy_history = zeros(num_steps, 1);
izz_history = zeros(num_steps, 1);
thrust_history = zeros(num_steps, 1);

% ===== 초기화 (첫 번째 호출) =====
fprintf('로켓 파라미터 초기화 중...\n');
% flatRocketParams를 직접 전달
params_init = vehicle_params(flatRocketParams, 0, motorCsvPath, true);  % reset=true로 초기화

% ===== 시뮬레이션 루프 =====
fprintf('시뮬레이션 루프 시작 (%d 스텝)...\n', num_steps);
for i = 1:num_steps
    t = time_steps(i);
    
    % 현재 시간의 로켓 파라미터 계산 (vehicle_params 사용)
    params = vehicle_params(flatRocketParams, t, motorCsvPath);
    
    % 결과 저장
    mass_history(i) = params.vehicle.m_W;
    cg_abs_history(i, :) = params.vehicle.CG_abs;
    cg_delta_history(i, :) = params.vehicle.CG_delta;
    ixx_history(i) = params.vehicle.J_X;
    iyy_history(i) = params.vehicle.J_Y;
    izz_history(i) = params.vehicle.J_Z;
    thrust_history(i) = params.vehicle.thrust;
    
    % 진행 상황 표시 (10% 간격)
    if mod(i, floor(num_steps/10)) == 0
        fprintf('진행: %.0f%% 완료 (시간=%.2fs, 질량=%.3fkg)\n', ...
                100*i/num_steps, t, params.vehicle.m_W);
    end
end
fprintf('시뮬레이션 완료\n');

% ===== 결과 플롯 =====
fprintf('결과 그래프 생성 중...\n');
figure('Name', '로켓 성능 종합 변화', 'NumberTitle', 'off', 'Position', [100, 100, 900, 600]);

subplot(2, 2, 1);
plot(time_steps, mass_history, 'LineWidth', 2);
grid on;
xlabel('시간 (s)');
ylabel('질량 (kg)');
title('질량 변화');

subplot(2, 2, 2);
plot(time_steps, thrust_history, 'LineWidth', 2);
grid on;
xlabel('시간 (s)');
ylabel('추력 (N)');
title('추력 변화');

subplot(2, 2, 3);
plot(time_steps, cg_abs_history(:,1), 'r-', 'LineWidth', 2);
grid on;
xlabel('시간 (s)');
ylabel('X축 CG 절대 위치 (m)');
title('무게중심 X축 절대 위치 변화');

subplot(2, 2, 4);
yyaxis left;
plot(time_steps, mass_history, 'b-', 'LineWidth', 2);
ylabel('질량 (kg)');
yyaxis right;
plot(time_steps, cg_delta_history(:,1), 'r-', 'LineWidth', 2);
ylabel('CG X축 변화량 (m)');
grid on;
xlabel('시간 (s)');
title('질량과 CG 변화량의 상관관계');

% ===== 추가 그래프: 관성 텐서 =====
figure('Name', '관성 텐서 변화', 'NumberTitle', 'off', 'Position', [100, 100, 900, 300]);

plot(time_steps, ixx_history, 'r-', 'LineWidth', 2); hold on;
plot(time_steps, iyy_history, 'g-', 'LineWidth', 2);
plot(time_steps, izz_history, 'b-', 'LineWidth', 2);
grid on;
xlabel('시간 (s)');
ylabel('관성 모멘트 (kg-m^2)');
title('관성 텐서 주 대각 성분');
legend('Ixx', 'Iyy', 'Izz');

% ===== 처음 10개 데이터 값 출력 (디버깅용) =====
fprintf('\n처음 10개 시간 스텝의 데이터:\n');
fprintf('시간(s)\t질량(kg)\tCG_X(m)\tIxx\t\tIyy\t\tIzz\n');
fprintf('------------------------------------------------------------------------\n');
for i = 1:min(10, num_steps)
    fprintf('%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n', ...
        time_steps(i), mass_history(i), ...
        cg_abs_history(i,1), ixx_history(i), iyy_history(i), izz_history(i));
end