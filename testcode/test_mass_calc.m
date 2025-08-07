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
cg_history = zeros(num_steps, 3);
ixx_history = zeros(num_steps, 1);
iyy_history = zeros(num_steps, 1);
izz_history = zeros(num_steps, 1);
thrust_history = zeros(num_steps, 1);

% ===== 첫 번째 호출에서 정적 부품 계산 및 캐싱 =====
fprintf('첫 번째 질량 특성 계산 (정적 부품 캐싱)...\n');
[~, ~, ~, components] = vehicle_mass_calc(flatRocketParams, 0, motorCsvPath);
static_cache = components{1};  % 정적 부품 캐시 저장

% ===== 시뮬레이션 루프 =====
fprintf('시뮬레이션 루프 시작 (%d 스텝)...\n', num_steps);
for i = 1:num_steps
    t = time_steps(i);
    
    % 로켓 질량 특성 계산 (캐시된 정적 부품 사용)
    [MT, CG, ICG] = vehicle_mass_calc(flatRocketParams, t, motorCsvPath, static_cache);
    
    % 현재 추력 값 보간
    current_thrust = interp1(timeArray, thrustArray, t, 'linear');
    if isnan(current_thrust)
        current_thrust = 0;
    end
    
    % 결과 저장
    mass_history(i) = MT;
    cg_history(i, :) = CG;
    ixx_history(i) = ICG(1,1);
    iyy_history(i) = ICG(2,2);
    izz_history(i) = ICG(3,3);
    thrust_history(i) = current_thrust;
    
    % 진행 상황 표시 (10% 간격)
    if mod(i, floor(num_steps/10)) == 0
        fprintf('진행: %.0f%% 완료\n', 100*i/num_steps);
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
plot(time_steps, cg_history(:,1), 'r-', 'LineWidth', 2);
grid on;
xlabel('시간 (s)');
ylabel('X축 CG 위치 (m)');
title('무게중심 X축 위치 변화');

subplot(2, 2, 4);
yyaxis left;
plot(time_steps, mass_history, 'b-', 'LineWidth', 2);
ylabel('질량 (kg)');
yyaxis right;
plot(time_steps, cg_history(:,1), 'r-', 'LineWidth', 2);
ylabel('X축 CG 위치 (m)');
grid on;
xlabel('시간 (s)');
title('질량과 CG 위치의 상관관계');