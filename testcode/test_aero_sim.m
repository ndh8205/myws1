%% 공력 계수 테스트 및 검증 메인 스크립트
% vehicle_params와 aero_coefficients를 사용한 공력 계수 분석

clear all;
close all;
clc;

% 경로 설정 (필요시 수정)
addpath(genpath('C:\Users\USER\Desktop\hanul_GNC'));

%% 1. 데이터 및 환경 설정
fprintf('데이터 및 환경 설정 중...\n');

% 파일 경로 설정
force_moment_path = 'csv_Force_moment_arg.csv';  % 정적 공력계수 파일
rkt_params_path = 'rocket.xml';                  % 로켓 파라미터 파일
motor_csv_path = 'AeroTech_M2400T.csv';          % 모터 파일

% 정적 공력계수 파일 읽기
try
    % 열 이름 보존 설정으로 CSV 파일 읽기
    opts = detectImportOptions(force_moment_path, 'VariableNamingRule', 'preserve');
    static_data = readtable(force_moment_path, opts);
    
    % 기본 정보 출력
    unique_alpha = unique(static_data.AoA);
    unique_mach = unique(static_data.Mach);
    fprintf('정적 계수 파일 로드 완료:\n');
    fprintf('  받음각: %d개 지점 (', length(unique_alpha));
    fprintf('%.1f ', unique_alpha);
    fprintf(')\n');
    fprintf('  마하수: %d개 지점 (', length(unique_mach));
    fprintf('%.2f ', unique_mach);
    fprintf(')\n');
    
    % 데이터 구조 확인
    fprintf('  CSV 파일 열 이름: %s\n', strjoin(static_data.Properties.VariableNames, ', '));
catch ME
    error('데이터 로드 실패: %s', ME.message);
end

% 로켓 파라미터 설정
try
    % 로켓 파라미터 로드
    rocketParams = extractRocketParams(rkt_params_path);
    flatRocketParams = flattenRocketStructure(rocketParams);
    params = vehicle_params(flatRocketParams, 0, motor_csv_path, true);
    
    fprintf('로켓 파라미터 로드 완료\n');
    fprintf('  로켓 직경: %.3f m\n', params.vehicle.D_ref);
    fprintf('  로켓 길이: %.3f m\n', params.vehicle.L);
    
    % 음속 필드 추가 (기본값 설정)
    if ~isfield(params.environment, 'sound_speed')
        params.environment.sound_speed = 340;  % 음속 기본값 [m/s]
        fprintf('  음속 필드 없음 - 기본값 설정: %.1f m/s\n', params.environment.sound_speed);
    end
catch ME
    % 오류 발생시 기본 파라미터 설정
    fprintf('로켓 파라미터 로드 실패: %s\n', ME.message);
    fprintf('기본 파라미터 사용\n');
    
    % 기본 파라미터 설정
    params = struct();
    params.vehicle = struct();
    params.vehicle.D_ref = 0.136;         % 로켓 직경 [m]
    params.vehicle.L = 2.679;             % 로켓 길이 [m]
    params.vehicle.S_A_ref = pi * (params.vehicle.D_ref/2)^2;  % 단면적 [m^2]
    params.environment = struct();
    params.environment.ro = 1.225;        % 대기 밀도 [kg/m^3]
    params.environment.sound_speed = 340; % 음속 [m/s]
end

%% 2. 분석 설정
fprintf('분석 설정 중...\n');

% 분석 범위 설정
alpha_range = 0:5:90;       % 받음각 범위 [도] - 0~90도만 분석
mach_range = [0.5, 0.8, 1.0, 1.2, 1.5, 2.0];  % 마하수 범위

% 테스트용 각속도 값
pm_test = 1.0;  % 롤 각속도 [rad/s]
qm_test = 1.0;  % 피치 각속도 [rad/s]
rm_test = 1.0;  % 요 각속도 [rad/s]

% 결과 저장 배열 초기화
n_alpha = length(alpha_range);
n_mach = length(mach_range);

% 정적 공력 계수 저장 배열
CAm_data = zeros(n_alpha, n_mach);
CYm_data = zeros(n_alpha, n_mach);
CNm_data = zeros(n_alpha, n_mach);
Clm_data = zeros(n_alpha, n_mach);
Cmm_data = zeros(n_alpha, n_mach);
Cyawm_data = zeros(n_alpha, n_mach);

% 댐핑 미분계수 저장 배열
Clpm_data = zeros(n_alpha, n_mach);
Cmqm_data = zeros(n_alpha, n_mach);
Cyawrm_data = zeros(n_alpha, n_mach);

% 댐핑 모멘트 계수 저장 배열 (테스트 각속도 기준)
Clmd_data = zeros(n_alpha, n_mach);
Cmmd_data = zeros(n_alpha, n_mach);
Cyawmd_data = zeros(n_alpha, n_mach);

%% 3. 공력 계수 객체 초기화
fprintf('공력 계수 객체 초기화 중...\n');
% 첫 번째 호출로 초기화 - 이 때 공력 데이터 로드됨
aero = aero_coefficients(params, 0, 1.0, pm_test, qm_test, rm_test, true);

%% 4. 공력 계수 계산 루프
fprintf('공력 계수 계산 중...\n');

% 진행 표시기 설정
total_steps = n_alpha * n_mach;
progress_step = max(1, floor(total_steps/20));  % 5% 단위로 진행상황 표시
progress_count = 0;

for i = 1:n_alpha
    for j = 1:n_mach
        % 현재 분석 지점
        alpha = alpha_range(i);
        mach = mach_range(j);
        
        % aero_coefficients 함수 호출로 공력 계수 계산
        aero = aero_coefficients(params, alpha, mach, pm_test, qm_test, rm_test);
        
        % 정적 공력 계수 저장
        CAm_data(i, j) = aero.static.CAm;
        CYm_data(i, j) = aero.static.CYm;
        CNm_data(i, j) = aero.static.CNm;
        Clm_data(i, j) = aero.static.Clm;
        Cmm_data(i, j) = aero.static.Cmm;
        Cyawm_data(i, j) = aero.static.Cyawm;
        
        % 댐핑 미분계수 저장
        Clpm_data(i, j) = aero.damping_derivatives.Clpm;
        Cmqm_data(i, j) = aero.damping_derivatives.Cmqm;
        Cyawrm_data(i, j) = aero.damping_derivatives.Cyawrm;
        
        % 댐핑 모멘트 계수 저장
        Clmd_data(i, j) = aero.damping_moments.Clmd;
        Cmmd_data(i, j) = aero.damping_moments.Cmmd;
        Cyawmd_data(i, j) = aero.damping_moments.Cyawmd;
        
        % 진행상황 표시
        progress_count = progress_count + 1;
        if mod(progress_count, progress_step) == 0 || progress_count == total_steps
            fprintf('  계산 진행: %.1f%% 완료 (α=%.1f°, Ma=%.2f)\n', ...
                   100*progress_count/total_steps, alpha, mach);
        end
    end
end

fprintf('공력 계수 계산 완료!\n');

%% 5. 결과 시각화 - 정적 계수 3D 플롯
fprintf('정적 계수 시각화 중...\n');

% 5.1 받음각-마하수에 따른 정적 계수 3D 시각화
figure('Name', '정적 공력계수 3D 시각화', 'Position', [100, 100, 1200, 600]);

% 메쉬 그리드 생성
[Alpha, Mach] = meshgrid(alpha_range, mach_range);

% CA 3D 서피스 플롯
subplot(2, 3, 1);
surf(Alpha', Mach', CAm_data);
xlabel('받음각 [도]');
ylabel('마하수');
zlabel('CAm');
title('축력 계수');
colorbar;
grid on;

% CN 3D 서피스 플롯
subplot(2, 3, 2);
surf(Alpha', Mach', CNm_data);
xlabel('받음각 [도]');
ylabel('마하수');
zlabel('CNm');
title('법선력 계수');
colorbar;
grid on;

% Cm 3D 서피스 플롯
subplot(2, 3, 3);
surf(Alpha', Mach', Cmm_data);
xlabel('받음각 [도]');
ylabel('마하수');
zlabel('Cmm');
title('피칭 모멘트 계수');
colorbar;
grid on;

% CY 3D 서피스 플롯
subplot(2, 3, 4);
surf(Alpha', Mach', CYm_data);
xlabel('받음각 [도]');
ylabel('마하수');
zlabel('CYm');
title('측력 계수');
colorbar;
grid on;

% Cl 3D 서피스 플롯
subplot(2, 3, 5);
surf(Alpha', Mach', Clm_data);
xlabel('받음각 [도]');
ylabel('마하수');
zlabel('Clm');
title('롤링 모멘트 계수');
colorbar;
grid on;

% Cyaw 3D 서피스 플롯
subplot(2, 3, 6);
surf(Alpha', Mach', Cyawm_data);
xlabel('받음각 [도]');
ylabel('마하수');
zlabel('Cyawm');
title('요잉 모멘트 계수');
colorbar;
grid on;

sgtitle('정적 공력계수 3D 시각화', 'FontSize', 14);

%% 6. 결과 시각화 - 댐핑 미분계수 3D 플롯
% 6.1 받음각-마하수에 따른 댐핑 미분계수 3D 시각화
figure('Name', '댐핑 미분계수 3D 시각화', 'Position', [100, 100, 1200, 400]);

% Clpm 3D 서피스 플롯
subplot(1, 3, 1);
surf(Alpha', Mach', Clpm_data);
xlabel('받음각 [도]');
ylabel('마하수');
zlabel('Clpm');
title('롤 댐핑 미분계수');
colorbar;
grid on;

% Cmqm 3D 서피스 플롯
subplot(1, 3, 2);
surf(Alpha', Mach', Cmqm_data);
xlabel('받음각 [도]');
ylabel('마하수');
zlabel('Cmqm');
title('피치 댐핑 미분계수');
colorbar;
grid on;

% Cyawrm 3D 서피스 플롯
subplot(1, 3, 3);
surf(Alpha', Mach', Cyawrm_data);
xlabel('받음각 [도]');
ylabel('마하수');
zlabel('Cyawrm');
title('요 댐핑 미분계수');
colorbar;
grid on;

sgtitle('댐핑 미분계수 3D 시각화', 'FontSize', 14);

%% 7. 결과 시각화 - 댐핑 계수 상세 분석
% 7.1 받음각에 따른 댐핑 미분계수 변화 (마하수별 곡선)
figure('Name', '받음각에 따른 댐핑 미분계수 변화', 'Position', [100, 100, 1200, 400]);

% 롤 댐핑 미분계수
subplot(1, 3, 1);
hold on;
for j = 1:n_mach
    plot(alpha_range, Clpm_data(:, j), 'o-', 'LineWidth', 2, 'DisplayName', sprintf('Ma = %.1f', mach_range(j)));
end
hold off;
grid on;
xlabel('받음각 [도]');
ylabel('Clpm');
title('롤 댐핑 미분계수');
legend('Location', 'best');

% 피치 댐핑 미분계수
subplot(1, 3, 2);
hold on;
for j = 1:n_mach
    plot(alpha_range, Cmqm_data(:, j), 'o-', 'LineWidth', 2, 'DisplayName', sprintf('Ma = %.1f', mach_range(j)));
end
hold off;
grid on;
xlabel('받음각 [도]');
ylabel('Cmqm');
title('피치 댐핑 미분계수');
legend('Location', 'best');

% 요 댐핑 미분계수
subplot(1, 3, 3);
hold on;
for j = 1:n_mach
    plot(alpha_range, Cyawrm_data(:, j), 'o-', 'LineWidth', 2, 'DisplayName', sprintf('Ma = %.1f', mach_range(j)));
end
hold off;
grid on;
xlabel('받음각 [도]');
ylabel('Cyawrm');
title('요 댐핑 미분계수');
legend('Location', 'best');

sgtitle('받음각에 따른 댐핑 미분계수 변화', 'FontSize', 14);

% 7.2 마하수에 따른 댐핑 미분계수 변화 (0도, 30도, 60도, 90도 받음각)
test_alphas = [0, 30, 60, 90];  % 테스트할 받음각
alpha_indices = zeros(size(test_alphas));

% 가장 가까운 인덱스 찾기
for i = 1:length(test_alphas)
    [~, alpha_indices(i)] = min(abs(alpha_range - test_alphas(i)));
end

figure('Name', '마하수에 따른 댐핑 미분계수 변화', 'Position', [100, 100, 1200, 400]);

% 롤 댐핑 미분계수
subplot(1, 3, 1);
hold on;
for i = 1:length(test_alphas)
    idx = alpha_indices(i);
    plot(mach_range, Clpm_data(idx, :), 'o-', 'LineWidth', 2, 'DisplayName', sprintf('\\alpha = %d°', test_alphas(i)));
end
hold off;
grid on;
xlabel('마하수');
ylabel('Clpm');
title('롤 댐핑 미분계수');
legend('Location', 'best');

% 피치 댐핑 미분계수
subplot(1, 3, 2);
hold on;
for i = 1:length(test_alphas)
    idx = alpha_indices(i);
    plot(mach_range, Cmqm_data(idx, :), 'o-', 'LineWidth', 2, 'DisplayName', sprintf('\\alpha = %d°', test_alphas(i)));
end
hold off;
grid on;
xlabel('마하수');
ylabel('Cmqm');
title('피치 댐핑 미분계수');
legend('Location', 'best');

% 요 댐핑 미분계수
subplot(1, 3, 3);
hold on;
for i = 1:length(test_alphas)
    idx = alpha_indices(i);
    plot(mach_range, Cyawrm_data(idx, :), 'o-', 'LineWidth', 2, 'DisplayName', sprintf('\\alpha = %d°', test_alphas(i)));
end
hold off;
grid on;
xlabel('마하수');
ylabel('Cyawrm');
title('요 댐핑 미분계수');
legend('Location', 'best');

sgtitle('마하수에 따른 댐핑 미분계수 변화', 'FontSize', 14);

%% 8. 댐핑 모멘트 계수 시각화
% 8.1 모멘트 계수 (Ma=1.0에서 받음각별)
mach_one_idx = find(abs(mach_range - 1.0) < 0.01, 1);
if isempty(mach_one_idx)
    mach_one_idx = ceil(n_mach/2);  % 중간값 기본 사용
end

figure('Name', '댐핑 모멘트 계수 (Ma=1.0)', 'Position', [100, 100, 900, 400]);
hold on;
plot(alpha_range, Clmd_data(:, mach_one_idx), 'ro-', 'LineWidth', 2, 'DisplayName', 'Clmd');
plot(alpha_range, Cmmd_data(:, mach_one_idx), 'go-', 'LineWidth', 2, 'DisplayName', 'Cmmd');
plot(alpha_range, Cyawmd_data(:, mach_one_idx), 'bo-', 'LineWidth', 2, 'DisplayName', 'Cyawmd');
hold off;
grid on;
xlabel('받음각 [도]');
ylabel('댐핑 모멘트 계수');
title(sprintf('마하 %.1f에서의 댐핑 모멘트 계수 (p,q,r = %.1f rad/s)', ...
              mach_range(mach_one_idx), pm_test));
legend('Location', 'best');

%% 9. 정적 계수와 댐핑 계수의 관계 분석
% 9.1 피치 모멘트 계수(Cmm)와 피치 댐핑 미분계수(Cmqm) 관계 시각화
figure('Name', '피치 모멘트 계수와 댐핑 계수 관계', 'Position', [100, 100, 1200, 400]);

% 피치 모멘트 계수와 받음각 관계
subplot(1, 3, 1);
hold on;
for j = 1:n_mach
    plot(alpha_range, Cmm_data(:, j), 'o-', 'LineWidth', 2, 'DisplayName', sprintf('Ma = %.1f', mach_range(j)));
end
hold off;
grid on;
xlabel('받음각 [도]');
ylabel('Cmm');
title('피치 모멘트 계수 vs 받음각');
legend('Location', 'best');

% 피치 모멘트 기울기 계산 (중앙 차분)
dCmm = zeros(n_alpha-1, n_mach);
for j = 1:n_mach
    dCmm(:, j) = diff(Cmm_data(:, j)) ./ diff(alpha_range' * pi/180);  % 라디안으로 변환
end
alpha_mid = (alpha_range(1:end-1) + alpha_range(2:end)) / 2;

% 피치 모멘트 기울기
subplot(1, 3, 2);
hold on;
for j = 1:n_mach
    plot(alpha_mid, dCmm(:, j), 'o-', 'LineWidth', 2, 'DisplayName', sprintf('Ma = %.1f', mach_range(j)));
end
hold off;
grid on;
xlabel('받음각 [도]');
ylabel('dCmm/dα [1/rad]');
title('피치 모멘트 기울기');
legend('Location', 'best');

% 피치 댐핑 미분계수
subplot(1, 3, 3);
hold on;
for j = 1:n_mach
    plot(alpha_range, Cmqm_data(:, j), 'o-', 'LineWidth', 2, 'DisplayName', sprintf('Ma = %.1f', mach_range(j)));
end
hold off;
grid on;
xlabel('받음각 [도]');
ylabel('Cmqm');
title('피치 댐핑 미분계수');
legend('Location', 'best');

sgtitle('피치 모멘트 계수와 댐핑 계수 관계', 'FontSize', 14);

%% 10. 결과 요약 출력
fprintf('\n==== 공력 계수 분석 결과 요약 ====\n');

fprintf('1. 정적 공력 계수 범위:\n');
fprintf('   CAm:    [%.4f, %.4f]\n', min(CAm_data(:)), max(CAm_data(:)));
fprintf('   CNm:    [%.4f, %.4f]\n', min(CNm_data(:)), max(CNm_data(:)));
fprintf('   Cmm:    [%.4f, %.4f]\n', min(Cmm_data(:)), max(Cmm_data(:)));

fprintf('\n2. 댐핑 미분계수 범위:\n');
fprintf('   Clpm:   [%.4f, %.4f]\n', min(Clpm_data(:)), max(Clpm_data(:)));
fprintf('   Cmqm:   [%.4f, %.4f]\n', min(Cmqm_data(:)), max(Cmqm_data(:)));
fprintf('   Cyawrm: [%.4f, %.4f]\n', min(Cyawrm_data(:)), max(Cyawrm_data(:)));

fprintf('\n3. 0° 받음각, 마하 1.0에서의 계수:\n');
[~, a0_idx] = min(abs(alpha_range - 0));
if isempty(mach_one_idx)
    [~, mach_one_idx] = min(abs(mach_range - 1.0));
end
fprintf('   CAm = %.4f, CNm = %.4f, Cmm = %.4f\n', ...
        CAm_data(a0_idx, mach_one_idx), CNm_data(a0_idx, mach_one_idx), Cmm_data(a0_idx, mach_one_idx));
fprintf('   Clpm = %.4f, Cmqm = %.4f, Cyawrm = %.4f\n', ...
        Clpm_data(a0_idx, mach_one_idx), Cmqm_data(a0_idx, mach_one_idx), Cyawrm_data(a0_idx, mach_one_idx));

fprintf('\n4. 댐핑 특성 요약:\n');
fprintf('   - 롤 댐핑: 받음각 증가에 따라 감소, 천음속에서 최대\n');
fprintf('   - 피치 댐핑: 받음각 변화에 따른 모멘트 기울기에 비례\n');
fprintf('   - 요 댐핑: 낮은 받음각에서 피치와 유사, 높은 받음각에서 감소\n');

fprintf('\n==== 분석 완료 ====\n');