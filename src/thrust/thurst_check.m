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

% 시각화
% 스테이지 1 시간에 따른 질량 및 추력 변화
figure;
yyaxis left;
plot(data_stage1.("Time(s)"), total_mass_stage1, '-o');
ylabel('Total Propellant Mass (g)');
yyaxis right;
plot(data_stage1.("Time(s)"), data_stage1.("Thrust(N)"), '-x');
ylabel('Thrust (N)');
xlabel('Time (s)');
title('Stage 1: Time vs Total Propellant Mass and Thrust');
legend('Total Propellant Mass', 'Thrust');

% 스테이지 2 시간에 따른 질량 및 추력 변화
figure;
yyaxis left;
plot(data_stage2.("Time(s)"), total_mass_stage2, '-o');
ylabel('Total Propellant Mass (g)');
yyaxis right;
plot(data_stage2.("Time(s)"), data_stage2.("Thrust(N)"), '-x');
ylabel('Thrust (N)');
xlabel('Time (s)');
title('Stage 2: Time vs Total Propellant Mass and Thrust');
legend('Total Propellant Mass', 'Thrust');

