clear all
clc
close all

% 데이터 로드
opts = detectImportOptions('stage1_ver1.csv', 'VariableNamingRule', 'preserve');
data_stage1 = readtable('stage1_ver1.csv', opts);
opts = detectImportOptions('stage2_ver1.csv', 'VariableNamingRule', 'preserve');
data_stage2 = readtable('stage2_ver1.csv', opts);

% 총 추진제 질량 계산
% 스테이지 1
total_mass_stage1 = data_stage1.("Propellant Mass(G1;g)") + data_stage1.("Propellant Mass(G2;g)") + ...
                    data_stage1.("Propellant Mass(G3;g)") + data_stage1.("Propellant Mass(G4;g)") + ...
                    data_stage1.("Propellant Mass(G5;g)") + data_stage1.("Propellant Mass(G6;g)") + ...
                    data_stage1.("Propellant Mass(G7;g)");

% 스테이지 2
total_mass_stage2 = data_stage2.("Propellant Mass(G1;g)") + data_stage2.("Propellant Mass(G2;g)") + ...
                    data_stage2.("Propellant Mass(G3;g)") + data_stage2.("Propellant Mass(G4;g)");

% 보간을 위한 새로운 시간 벡터 생성
time_stage1 = data_stage1.("Time(s)")';
time_stage2 = data_stage2.("Time(s)")';

new_time_stage1 = linspace(min(time_stage1), max(time_stage1), 1000); % 1000개의 점으로 보간
new_time_stage2 = linspace(min(time_stage2), max(time_stage2), 1000); % 1000개의 점으로 보간

% 보간 수행
interp_mass_stage1 = interp1(time_stage1, total_mass_stage1 , new_time_stage1, 'linear')';
interp_thrust_stage1 = interp1(time_stage1, data_stage1.("Thrust(N)"), new_time_stage1, 'linear')';
interp_mass_stage2 = interp1(time_stage2, total_mass_stage2 , new_time_stage2, 'linear')';
interp_thrust_stage2 = interp1(time_stage2, data_stage2.("Thrust(N)"), new_time_stage2, 'linear')';

% 시각화
% 스테이지 1 시간에 따른 질량 및 추력 변화
figure;
yyaxis left;
plot(time_stage1, total_mass_stage1, 'o');
hold on;
plot(new_time_stage1, interp_mass_stage1, '-');
ylabel('Total Propellant Mass (g)');
yyaxis right;
plot(time_stage1, data_stage1.("Thrust(N)"), 'x');
hold on;
plot(new_time_stage1, interp_thrust_stage1, '-');
ylabel('Thrust (N)');
xlabel('Time (s)');
title('Stage 1: Time vs Total Propellant Mass and Thrust');
legend('Total Propellant Mass (original)', 'Total Propellant Mass (interpolated)', 'Thrust (original)', 'Thrust (interpolated)');

% 스테이지 2 시간에 따른 질량 및 추력 변화
figure;
yyaxis left;
plot(time_stage2, total_mass_stage2, 'o');
hold on;
plot(new_time_stage2, interp_mass_stage2, '-');
ylabel('Total Propellant Mass (g)');
yyaxis right;
plot(time_stage2, data_stage2.("Thrust(N)"), 'x');
hold on;
plot(new_time_stage2, interp_thrust_stage2, '-');
ylabel('Thrust (N)');
xlabel('Time (s)');
title('Stage 2: Time vs Total Propellant Mass and Thrust');
legend('Total Propellant Mass (original)', 'Total Propellant Mass (interpolated)', 'Thrust (original)', 'Thrust (interpolated)');
