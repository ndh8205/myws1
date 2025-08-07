clear all;
close all;
clc;
addpath(genpath('C:\Users\USER\Desktop\hanul_GNC'));

% XML → 파라미터 트리
xmlFilePath = 'rocket.xml';
rocketParams = extractRocketParams(xmlFilePath);
flatRocketParams = flattenRocketStructure(rocketParams);

% 파라미터 → (mass, cg, inertia) 리스트
compList_static = static_param(flatRocketParams);

% 모터 데이터 읽기
motorCsvPath = 'AeroTech_M2400T.csv';
opts = detectImportOptions(motorCsvPath, 'VariableNamingRule', 'preserve');
motorData = readtable(motorCsvPath, opts);
timeArray = motorData.("Time(s)");
thrustArray = motorData.("Thrust(N)");
simTime = max(timeArray);
dt = 1/400;

% 결과 저장 배열 초기화
time_steps = 0:dt:simTime;
num_steps = length(time_steps);
mass_history = zeros(num_steps, 1);
cg_history = zeros(num_steps, 3);
ixx_history = zeros(num_steps, 1);
iyy_history = zeros(num_steps, 1);
izz_history = zeros(num_steps, 1);
thrust_history = zeros(num_steps, 1);

% 시뮬레이션 루프
for i = 1:num_steps
    t = time_steps(i);
    
    % 모터 상태 계산
    compList_motor = motor_state(flatRocketParams, t, motorCsvPath);
    
    % 전체 질량 특성 계산
    combined_cell = {compList_static, compList_motor};
    [MT, CG, ICG] = getAssemblyProps(combined_cell);
    
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

end

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