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

% 상수 정의
g = 9.81;  % 중력가속도 (m/s^2)
dry_mass_stage1 = 6000 - max(interp_mass_stage1);  % 1단 건조 질량 (g)
dry_mass_stage2 = 2000 - max(interp_mass_stage2);  % 2단 건조 질량 (g, 추정치)

% 시뮬레이션 변수 초기화
velocity = 0;
height = 0;
mass = dry_mass_stage1 + max(interp_mass_stage1) + dry_mass_stage2 + max(interp_mass_stage2);

% 결과 저장을 위한 배열
velocities = zeros(size(new_time_stage1));
heights = zeros(size(new_time_stage1));
times = new_time_stage1;

% 1단 시뮬레이션
for i = 1:length(new_time_stage1)
    dt = new_time_stage1(2) - new_time_stage1(1);
    current_mass = dry_mass_stage1 + interp_mass_stage1(i) + dry_mass_stage2 + max(interp_mass_stage2);
    
    % 가속도 계산
    a = (interp_thrust_stage1(i)) / (current_mass) - g;
    
    % 속도와 고도 갱신
    velocity = velocity + a * dt;
    height = height + velocity * dt;
    
    % 결과 저장
    velocities(i) = velocity;
    heights(i) = height;
    
    % 추진제 모두 소모 시 1단 분리
    if interp_mass_stage1(i) <= 0
        break;
    end
end

% 2단 시뮬레이션
start_index_stage2 = i;
mass = dry_mass_stage2 + max(interp_mass_stage2);

for i = 1:length(new_time_stage2)
    dt = new_time_stage2(2) - new_time_stage2(1);
    current_mass = dry_mass_stage2 + interp_mass_stage2(i);
    
    % 가속도 계산
    a = (interp_thrust_stage2(i)) / (current_mass) - g;
    
    % 속도와 고도 갱신
    velocity = velocity + a * dt;
    height = height + velocity * dt;
    
    % 결과 저장
    index = start_index_stage2 + i - 1;
    if index <= length(velocities)
        velocities(index) = velocity;
        heights(index) = height;
        times(index) = new_time_stage1(end) + new_time_stage2(i);
    else
        velocities(end+1) = velocity;
        heights(end+1) = height;
        times(end+1) = new_time_stage1(end) + new_time_stage2(i);
    end
    
    % 추진제 모두 소모 또는 속도가 음수가 되면 종료
    if interp_mass_stage2(i) <= 0 || velocity < 0
        break;
    end
end

% 최대 고도 찾기
[max_height, max_height_index] = max(heights);
time_to_max_height = times(max_height_index);

% 결과 출력
fprintf('최대 고도: %.2f m\n', max_height);
fprintf('최대 고도 도달 시간: %.2f s\n', time_to_max_height);
fprintf('최대 속도: %.2f m/s\n', max(velocities));

% 그래프 그리기
figure;
subplot(2,1,1);
plot(times, heights);
title('고도 vs 시간');
xlabel('시간 (s)');
ylabel('고도 (m)');

subplot(2,1,2);
plot(times, velocities);
title('속도 vs 시간');
xlabel('시간 (s)');
ylabel('속도 (m/s)');