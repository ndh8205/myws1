% 초기화
clear all
close all
clc

% 데이터 로드
fprintf('공력 데이터 로딩 중...\n');
force_moment_path = 'csv_Force_moment_axiseq.csv';
opts = detectImportOptions(force_moment_path, 'VariableNamingRule', 'preserve');
aero_data = readtable(force_moment_path, opts);

% 좌표계 변환 - CFD 해석 X축 방향 반대 보정
aero_data.CA = -aero_data.CA;         % 축력 계수 반전
aero_data.Clm = -aero_data.Clm;       % 롤링 모멘트 계수 반전
aero_data.CN = -aero_data.CN;         % 법선력 계수 반전
aero_data.Cmm = -aero_data.Cmm;       % 피칭 모멘트 계수 반전
aero_data.Cyawm = -aero_data.Cyawm;   % 요잉 모멘트 계수 반전

% 실제 존재하는 받음각과 마하수 범위 추출
alpha_range = unique(aero_data.AoA);
mach_range = unique(aero_data.Mach);

% 실제 존재하는 마하수 값 출력
fprintf('실제 존재하는 마하수 값: ');
fprintf('%.2f ', mach_range);
fprintf('\n');

fprintf('공력 데이터 로드 완료: %d개 받음각, %d개 마하수\n', ...
        length(alpha_range), length(mach_range));

% 데이터 구조 확인
disp('데이터 미리보기:');
disp(head(aero_data, 5));

% 선택할 받음각들 (실제 데이터에서 추출)
if length(alpha_range) >= 4
    selected_alphas = alpha_range(1:4:end);
    if length(selected_alphas) > 6
        selected_alphas = selected_alphas(1:6);
    end
else
    selected_alphas = alpha_range;
end

% 선택할 마하수들 (실제 데이터에서 추출)
if length(mach_range) >= 4
    selected_machs = mach_range(1:max(1, floor(length(mach_range)/4)):end);
    if length(selected_machs) > 6
        selected_machs = selected_machs(1:6);
    end
else
    selected_machs = mach_range;
end

fprintf('선택된 받음각 값: ');
fprintf('%.1f ', selected_alphas);
fprintf('\n');

fprintf('선택된 마하수 값: ');
fprintf('%.2f ', selected_machs);
fprintf('\n');

colors = {'b', 'r', 'g', 'm', 'c', 'k'};
markers = {'o', 's', 'd', '^', 'v', 'p'};

% === 속도 미분계수 계산 및 시각화 ===
fprintf('\n===== 속도 미분계수 계산 및 시각화 =====\n');

% 로켓 파라미터 설정 (가상 파라미터, 실제 값은 필요에 따라 조정)
params = struct();
params.vehicle = struct();
params.vehicle.D_ref = 0.2;           % 기준 직경 [m]
params.vehicle.L = 2.0;               % 로켓 길이 [m]
params.environment = struct();
params.environment.sound_speed = 340; % 음속 [m/s]
params.position = [0, 0, -1000];      % 위치 [m] (고도 1000m 가정)

% 각속도 (0으로 가정)
pm = 0; qm = 0; rm = 0;

% 더 세밀한 마하수 범위 생성
mach_fine = linspace(min(mach_range), max(mach_range), 50);

% 속도 미분계수 계산을 위한 배열 초기화
CXu_array = zeros(length(selected_alphas), length(mach_fine));
CYv_array = zeros(length(selected_alphas), length(mach_fine));
CZw_array = zeros(length(selected_alphas), length(mach_fine));
Clp_array = zeros(length(selected_alphas), length(mach_fine));
Cmq_array = zeros(length(selected_alphas), length(mach_fine));
Cnr_array = zeros(length(selected_alphas), length(mach_fine));

% 결과 저장을 위한 테이블 배열
velocity_derivatives_tables = cell(length(selected_alphas), 1);

% CSV 파일 기반 미분계수 계산을 위한 보간 객체 생성
% scatteredInterpolant 대신 griddata 사용 (더 일반적인 보간 가능)
fprintf('미분계수 계산을 위한 공력 데이터 준비 중...\n');

% 3D 미분 계산용 보간 객체 생성
fprintf('CSV 데이터에서 직접 미분계수 계산을 위한 준비 중...\n');

% 데이터 메쉬 생성용 그리드
[ALPHA_MESH, MACH_MESH] = meshgrid(alpha_range, mach_range);
ALPHA_FLAT = ALPHA_MESH(:);
MACH_FLAT = MACH_MESH(:);

% 각 받음각과 마하수 조합에 대한 공력 계수 추출
CA_mesh = nan(size(ALPHA_MESH));
CY_mesh = nan(size(ALPHA_MESH));
CN_mesh = nan(size(ALPHA_MESH));
Clm_mesh = nan(size(ALPHA_MESH));
Cmm_mesh = nan(size(ALPHA_MESH));
Cyawm_mesh = nan(size(ALPHA_MESH));

% 메쉬에 데이터 채우기
for i = 1:length(mach_range)
    for j = 1:length(alpha_range)
        % 현재 마하수와 받음각에 해당하는 데이터 찾기
        idx = (aero_data.Mach == mach_range(i)) & (aero_data.AoA == alpha_range(j));
        
        if any(idx)
            CA_mesh(i, j) = aero_data.CA(idx);
            CY_mesh(i, j) = aero_data.CY(idx);
            CN_mesh(i, j) = aero_data.CN(idx);
            Clm_mesh(i, j) = aero_data.Clm(idx);
            Cmm_mesh(i, j) = aero_data.Cmm(idx);
            Cyawm_mesh(i, j) = aero_data.Cyawm(idx);
        end
    end
end

% 각 받음각에 대해 다양한 마하수에서 속도 미분계수 계산
for a = 1:length(selected_alphas)
    alpha_val = selected_alphas(a);
    fprintf('받음각 %.1f도에서 속도 미분계수 계산 중...\n', alpha_val);
    
    % 현재 받음각에 대한 결과 테이블 생성
    velocity_table = table();
    velocity_table.Mach = mach_fine';
    velocity_table.Velocity = mach_fine' * params.environment.sound_speed;
    
    % 각 마하수에 대해 계산
    for m = 1:length(mach_fine)
        mach_val = mach_fine(m);
        
        % 현재 조건에서의 정적 공력 계수 계산 (보간 사용)
        [CAm, CYm, CNm, Clm, Cmm, Cyawm] = interpolateStaticCoef(aero_data, alpha_val, mach_val, alpha_range, mach_range);
        
        % 롤, 피치, 요 댐핑 계수
        alpha_rad = alpha_val * pi/180;
        Clpm = -0.3 * cos(alpha_rad)^2; % 롤 댐핑 계수
        Cmqm = -1.5 * (cos(alpha_rad)^2 + 0.2); % 피치 댐핑 계수
        Cyawrm = Cmqm * (abs(cos(alpha_rad)) + 0.15); % 요 댐핑 계수
        
        % 정확한 미분계수 계산 - CSV 데이터 기반
        [CXu, CYv, CZw, Clp, Cmq, Cnr] = calcVelocityDerivatives_accurate(params, alpha_val, mach_val, ALPHA_MESH, MACH_MESH, CA_mesh, CY_mesh, CN_mesh, Clpm, Cmqm, Cyawrm);
        
        % 결과 저장
        CXu_array(a, m) = CXu;
        CYv_array(a, m) = CYv;
        CZw_array(a, m) = CZw;
        Clp_array(a, m) = Clp;
        Cmq_array(a, m) = Cmq;
        Cnr_array(a, m) = Cnr;
    end
    
    % 테이블에 결과 추가
    velocity_table.CXu = CXu_array(a, :)';
    velocity_table.CYv = CYv_array(a, :)';
    velocity_table.CZw = CZw_array(a, :)';
    velocity_table.Clp = Clp_array(a, :)';
    velocity_table.Cmq = Cmq_array(a, :)';
    velocity_table.Cnr = Cnr_array(a, :)';
    
    % 결과 테이블 저장
    velocity_derivatives_tables{a} = velocity_table;
    
    % 간단한 통계 출력
    fprintf('  CXu 범위: %.4f ~ %.4f\n', min(CXu_array(a, :)), max(CXu_array(a, :)));
    fprintf('  CYv 범위: %.4f ~ %.4f\n', min(CYv_array(a, :)), max(CYv_array(a, :)));
    fprintf('  CZw 범위: %.4f ~ %.4f\n', min(CZw_array(a, :)), max(CZw_array(a, :)));
end

% 전체 데이터 테이블에 저장
full_results = table();
row = 1;

for a = 1:length(selected_alphas)
    for m = 1:length(mach_fine)
        full_results.AoA(row) = selected_alphas(a);
        full_results.Mach(row) = mach_fine(m);
        full_results.Velocity(row) = mach_fine(m) * params.environment.sound_speed;
        
        % 속도 미분계수
        full_results.CXu(row) = CXu_array(a, m);
        full_results.CYv(row) = CYv_array(a, m);
        full_results.CZw(row) = CZw_array(a, m);
        full_results.Clp(row) = Clp_array(a, m);
        full_results.Cmq(row) = Cmq_array(a, m);
        full_results.Cnr(row) = Cnr_array(a, m);
        
        row = row + 1;
    end
end

% CSV 저장
writetable(full_results, 'velocity_derivatives.csv');
fprintf('결과가 velocity_derivatives.csv 파일에 저장되었습니다.\n');

% 결과 시각화
% 1. 속도 미분계수 (CXu, CYv, CZw) - 마하수에 따른 변화
figure('Name', '마하수에 따른 속도 미분계수 (힘 계수)', 'Position', [100, 100, 1200, 400]);

% CXu 그래프
subplot(1, 3, 1);
hold on;
for i = 1:length(selected_alphas)
    color_idx = mod(i-1, length(colors))+1;
    marker_idx = mod(i-1, length(markers))+1;
    
    % 결과 플롯
    plot(mach_fine, CXu_array(i, :), ...
         [colors{color_idx}, '-'], 'LineWidth', 2, 'DisplayName', sprintf('AoA = %.1f°', selected_alphas(i)));
    
    % 데이터 포인트 위치에 마커 추가
    for j = 1:length(selected_machs)
        [~, idx] = min(abs(mach_fine - selected_machs(j)));
        plot(mach_fine(idx), CXu_array(i, idx), ...
             [colors{color_idx}, markers{marker_idx}], 'MarkerSize', 8);
    end
end
hold off;
xlabel('마하수 (Mach)');
ylabel('축력 속도 미분계수 (CXu)');
title('마하수에 따른 축력 속도 미분계수 (CXu)');
grid on;
legend('Location', 'best');

% CYv 그래프
subplot(1, 3, 2);
hold on;
for i = 1:length(selected_alphas)
    color_idx = mod(i-1, length(colors))+1;
    marker_idx = mod(i-1, length(markers))+1;
    
    % 결과 플롯
    plot(mach_fine, CYv_array(i, :), ...
         [colors{color_idx}, '-'], 'LineWidth', 2, 'DisplayName', sprintf('AoA = %.1f°', selected_alphas(i)));
    
    % 데이터 포인트 위치에 마커 추가
    for j = 1:length(selected_machs)
        [~, idx] = min(abs(mach_fine - selected_machs(j)));
        plot(mach_fine(idx), CYv_array(i, idx), ...
             [colors{color_idx}, markers{marker_idx}], 'MarkerSize', 8);
    end
end
hold off;
xlabel('마하수 (Mach)');
ylabel('측력 속도 미분계수 (CYv)');
title('마하수에 따른 측력 속도 미분계수 (CYv)');
grid on;
legend('Location', 'best');

% CZw 그래프
subplot(1, 3, 3);
hold on;
for i = 1:length(selected_alphas)
    color_idx = mod(i-1, length(colors))+1;
    marker_idx = mod(i-1, length(markers))+1;
    
    % 결과 플롯
    plot(mach_fine, CZw_array(i, :), ...
         [colors{color_idx}, '-'], 'LineWidth', 2, 'DisplayName', sprintf('AoA = %.1f°', selected_alphas(i)));
    
    % 데이터 포인트 위치에 마커 추가
    for j = 1:length(selected_machs)
        [~, idx] = min(abs(mach_fine - selected_machs(j)));
        plot(mach_fine(idx), CZw_array(i, idx), ...
             [colors{color_idx}, markers{marker_idx}], 'MarkerSize', 8);
    end
end
hold off;
xlabel('마하수 (Mach)');
ylabel('법선력 속도 미분계수 (CZw)');
title('마하수에 따른 법선력 속도 미분계수 (CZw)');
grid on;
legend('Location', 'best');

% 2. 회전 속도 미분계수 (Clp, Cmq, Cnr) - 마하수에 따른 변화
figure('Name', '마하수에 따른 속도 미분계수 (모멘트 계수)', 'Position', [100, 600, 1200, 400]);

% Clp 그래프
subplot(1, 3, 1);
hold on;
for i = 1:length(selected_alphas)
    color_idx = mod(i-1, length(colors))+1;
    marker_idx = mod(i-1, length(markers))+1;
    
    % 결과 플롯
    plot(mach_fine, Clp_array(i, :), ...
         [colors{color_idx}, '-'], 'LineWidth', 2, 'DisplayName', sprintf('AoA = %.1f°', selected_alphas(i)));
    
    % 데이터 포인트 위치에 마커 추가
    for j = 1:length(selected_machs)
        [~, idx] = min(abs(mach_fine - selected_machs(j)));
        plot(mach_fine(idx), Clp_array(i, idx), ...
             [colors{color_idx}, markers{marker_idx}], 'MarkerSize', 8);
    end
end
hold off;
xlabel('마하수 (Mach)');
ylabel('롤 속도 미분계수 (Clp)');
title('마하수에 따른 롤 속도 미분계수 (Clp)');
grid on;
legend('Location', 'best');

% Cmq 그래프
subplot(1, 3, 2);
hold on;
for i = 1:length(selected_alphas)
    color_idx = mod(i-1, length(colors))+1;
    marker_idx = mod(i-1, length(markers))+1;
    
    % 결과 플롯
    plot(mach_fine, Cmq_array(i, :), ...
         [colors{color_idx}, '-'], 'LineWidth', 2, 'DisplayName', sprintf('AoA = %.1f°', selected_alphas(i)));
    
    % 데이터 포인트 위치에 마커 추가
    for j = 1:length(selected_machs)
        [~, idx] = min(abs(mach_fine - selected_machs(j)));
        plot(mach_fine(idx), Cmq_array(i, idx), ...
             [colors{color_idx}, markers{marker_idx}], 'MarkerSize', 8);
    end
end
hold off;
xlabel('마하수 (Mach)');
ylabel('피치 속도 미분계수 (Cmq)');
title('마하수에 따른 피치 속도 미분계수 (Cmq)');
grid on;
legend('Location', 'best');

% Cnr 그래프
subplot(1, 3, 3);
hold on;
for i = 1:length(selected_alphas)
    color_idx = mod(i-1, length(colors))+1;
    marker_idx = mod(i-1, length(markers))+1;
    
    % 결과 플롯
    plot(mach_fine, Cnr_array(i, :), ...
         [colors{color_idx}, '-'], 'LineWidth', 2, 'DisplayName', sprintf('AoA = %.1f°', selected_alphas(i)));
    
    % 데이터 포인트 위치에 마커 추가
    for j = 1:length(selected_machs)
        [~, idx] = min(abs(mach_fine - selected_machs(j)));
        plot(mach_fine(idx), Cnr_array(i, idx), ...
             [colors{color_idx}, markers{marker_idx}], 'MarkerSize', 8);
    end
end
hold off;
xlabel('마하수 (Mach)');
ylabel('요 속도 미분계수 (Cnr)');
title('마하수에 따른 요 속도 미분계수 (Cnr)');
grid on;
legend('Location', 'best');

% 3. 실제 속도 (m/s)에 따른 미분계수 변화 시각화
figure('Name', '속도에 따른 미분계수 (힘 계수)', 'Position', [100, 100, 1200, 400]);

% 속도 배열 (마하수 * 음속)
velocity_array = mach_fine * params.environment.sound_speed;

% CXu 그래프
subplot(1, 3, 1);
hold on;
for i = 1:length(selected_alphas)
    color_idx = mod(i-1, length(colors))+1;
    marker_idx = mod(i-1, length(markers))+1;
    
    % 결과 플롯
    plot(velocity_array, CXu_array(i, :), ...
         [colors{color_idx}, '-'], 'LineWidth', 2, 'DisplayName', sprintf('AoA = %.1f°', selected_alphas(i)));
    
    % 데이터 포인트 위치에 마커 추가
    for j = 1:length(selected_machs)
        [~, idx] = min(abs(mach_fine - selected_machs(j)));
        v = selected_machs(j) * params.environment.sound_speed;
        plot(v, CXu_array(i, idx), ...
             [colors{color_idx}, markers{marker_idx}], 'MarkerSize', 8);
    end
end
hold off;
xlabel('속도 (m/s)');
ylabel('축력 속도 미분계수 (CXu)');
title('속도에 따른 축력 속도 미분계수 (CXu)');
grid on;
legend('Location', 'best');

% CYv 그래프
subplot(1, 3, 2);
hold on;
for i = 1:length(selected_alphas)
    color_idx = mod(i-1, length(colors))+1;
    marker_idx = mod(i-1, length(markers))+1;
    
    % 결과 플롯
    plot(velocity_array, CYv_array(i, :), ...
         [colors{color_idx}, '-'], 'LineWidth', 2, 'DisplayName', sprintf('AoA = %.1f°', selected_alphas(i)));
    
    % 데이터 포인트 위치에 마커 추가
    for j = 1:length(selected_machs)
        [~, idx] = min(abs(mach_fine - selected_machs(j)));
        v = selected_machs(j) * params.environment.sound_speed;
        plot(v, CYv_array(i, idx), ...
             [colors{color_idx}, markers{marker_idx}], 'MarkerSize', 8);
    end
end
hold off;
xlabel('속도 (m/s)');
ylabel('측력 속도 미분계수 (CYv)');
title('속도에 따른 측력 속도 미분계수 (CYv)');
grid on;
legend('Location', 'best');

% CZw 그래프
subplot(1, 3, 3);
hold on;
for i = 1:length(selected_alphas)
    color_idx = mod(i-1, length(colors))+1;
    marker_idx = mod(i-1, length(markers))+1;
    
    % 결과 플롯
    plot(velocity_array, CZw_array(i, :), ...
         [colors{color_idx}, '-'], 'LineWidth', 2, 'DisplayName', sprintf('AoA = %.1f°', selected_alphas(i)));
    
    % 데이터 포인트 위치에 마커 추가
    for j = 1:length(selected_machs)
        [~, idx] = min(abs(mach_fine - selected_machs(j)));
        v = selected_machs(j) * params.environment.sound_speed;
        plot(v, CZw_array(i, idx), ...
             [colors{color_idx}, markers{marker_idx}], 'MarkerSize', 8);
    end
end
hold off;
xlabel('속도 (m/s)');
ylabel('법선력 속도 미분계수 (CZw)');
title('속도에 따른 법선력 속도 미분계수 (CZw)');
grid on;
legend('Location', 'best');

% 4. 실제 속도 (m/s)에 따른 회전 미분계수 변화 시각화
figure('Name', '속도에 따른 미분계수 (모멘트 계수)', 'Position', [100, 600, 1200, 400]);

% Clp 그래프
subplot(1, 3, 1);
hold on;
for i = 1:length(selected_alphas)
    color_idx = mod(i-1, length(colors))+1;
    marker_idx = mod(i-1, length(markers))+1;
    
    % 결과 플롯
    plot(velocity_array, Clp_array(i, :), ...
         [colors{color_idx}, '-'], 'LineWidth', 2, 'DisplayName', sprintf('AoA = %.1f°', selected_alphas(i)));
    
    % 데이터 포인트 위치에 마커 추가
    for j = 1:length(selected_machs)
        [~, idx] = min(abs(mach_fine - selected_machs(j)));
        v = selected_machs(j) * params.environment.sound_speed;
        plot(v, Clp_array(i, idx), ...
             [colors{color_idx}, markers{marker_idx}], 'MarkerSize', 8);
    end
end
hold off;
xlabel('속도 (m/s)');
ylabel('롤 속도 미분계수 (Clp)');
title('속도에 따른 롤 속도 미분계수 (Clp)');
grid on;
legend('Location', 'best');

% Cmq 그래프
subplot(1, 3, 2);
hold on;
for i = 1:length(selected_alphas)
    color_idx = mod(i-1, length(colors))+1;
    marker_idx = mod(i-1, length(markers))+1;
    
    % 결과 플롯
    plot(velocity_array, Cmq_array(i, :), ...
         [colors{color_idx}, '-'], 'LineWidth', 2, 'DisplayName', sprintf('AoA = %.1f°', selected_alphas(i)));
    
    % 데이터 포인트 위치에 마커 추가
    for j = 1:length(selected_machs)
        [~, idx] = min(abs(mach_fine - selected_machs(j)));
        v = selected_machs(j) * params.environment.sound_speed;
        plot(v, Cmq_array(i, idx), ...
             [colors{color_idx}, markers{marker_idx}], 'MarkerSize', 8);
    end
end
hold off;
xlabel('속도 (m/s)');
ylabel('피치 속도 미분계수 (Cmq)');
title('속도에 따른 피치 속도 미분계수 (Cmq)');
grid on;
legend('Location', 'best');

% Cnr 그래프
subplot(1, 3, 3);
hold on;
for i = 1:length(selected_alphas)
    color_idx = mod(i-1, length(colors))+1;
    marker_idx = mod(i-1, length(markers))+1;
    
    % 결과 플롯
    plot(velocity_array, Cnr_array(i, :), ...
         [colors{color_idx}, '-'], 'LineWidth', 2, 'DisplayName', sprintf('AoA = %.1f°', selected_alphas(i)));
    
    % 데이터 포인트 위치에 마커 추가
    for j = 1:length(selected_machs)
        [~, idx] = min(abs(mach_fine - selected_machs(j)));
        v = selected_machs(j) * params.environment.sound_speed;
        plot(v, Cnr_array(i, idx), ...
             [colors{color_idx}, markers{marker_idx}], 'MarkerSize', 8);
    end
end
hold off;
xlabel('속도 (m/s)');
ylabel('요 속도 미분계수 (Cnr)');
title('속도에 따른 요 속도 미분계수 (Cnr)');
grid on;
legend('Location', 'best');

% 데이터 통계 요약
fprintf('\n==== 속도 미분계수 통계 요약 ====\n');
fprintf('CXu 범위: %.4f ~ %.4f\n', min(CXu_array(:)), max(CXu_array(:)));
fprintf('CYv 범위: %.4f ~ %.4f\n', min(CYv_array(:)), max(CYv_array(:)));
fprintf('CZw 범위: %.4f ~ %.4f\n', min(CZw_array(:)), max(CZw_array(:)));
fprintf('Clp 범위: %.4f ~ %.4f\n', min(Clp_array(:)), max(Clp_array(:)));
fprintf('Cmq 범위: %.4f ~ %.4f\n', min(Cmq_array(:)), max(Cmq_array(:)));
fprintf('Cnr 범위: %.4f ~ %.4f\n', min(Cnr_array(:)), max(Cnr_array(:)));

%% 두 번째 코드에서 가져온 보간 함수 추가
function [CAm, CYm, CNm, Clm, Cmm, Cyawm] = interpolateStaticCoef(aero_data, alpha, mach, alpha_range, mach_range)
    % 2D 선형 보간법을 사용한 정적 공력 계수 계산 함수
    
    % 결과값 초기화
    CAm = 0.5; CYm = 0; CNm = 0; Clm = 0; Cmm = 0; Cyawm = 0;
    
    % 데이터 범위 확인
    if alpha < min(alpha_range) || alpha > max(alpha_range) || ...
       mach < min(mach_range) || mach > max(mach_range)
        % 범위를 벗어나면 경계값으로 제한
        alpha = max(min(alpha, max(alpha_range)), min(alpha_range));
        mach = max(min(mach, max(mach_range)), min(mach_range));
    end
    
    % 주변 데이터 포인트 찾기
    alpha_idx_low = find(alpha_range <= alpha, 1, 'last');
    alpha_idx_high = find(alpha_range > alpha, 1, 'first');
    
    mach_idx_low = find(mach_range <= mach, 1, 'last');
    mach_idx_high = find(mach_range > mach, 1, 'first');
    
    % 경계 처리
    if isempty(alpha_idx_low), alpha_idx_low = 1; end
    if isempty(alpha_idx_high), alpha_idx_high = length(alpha_range); end
    if isempty(mach_idx_low), mach_idx_low = 1; end
    if isempty(mach_idx_high), mach_idx_high = length(mach_range); end
    
    % 일치하는 경우(보간 불필요)
    if alpha_idx_low == alpha_idx_high && mach_idx_low == mach_idx_high
        % 정확히 일치하는 데이터 포인트 찾기
        mask = (aero_data.AoA == alpha_range(alpha_idx_low)) & ...
               (aero_data.Mach == mach_range(mach_idx_low));
        
        if any(mask)
            row = aero_data(mask, :);
            CAm = row.CA(1);
            CYm = row.CY(1);
            CNm = row.CN(1);
            
            if ismember('Clm', row.Properties.VariableNames)
                Clm = row.Clm(1);
            end
            
            if ismember('Cmm', row.Properties.VariableNames)
                Cmm = row.Cmm(1);
            end
            
            if ismember('Cyawm', row.Properties.VariableNames)
                Cyawm = row.Cyawm(1);
            end
        end
        return;
    end
    
    % 2D 선형 보간에 필요한 4개 데이터 포인트 값 가져오기
    alpha_low = alpha_range(alpha_idx_low);
    alpha_high = alpha_range(alpha_idx_high);
    mach_low = mach_range(mach_idx_low);
    mach_high = mach_range(mach_idx_high);
    
    % 4개의 격자점 데이터
    CA_points = zeros(2, 2);
    CY_points = zeros(2, 2);
    CN_points = zeros(2, 2);
    Clm_points = zeros(2, 2);
    Cmm_points = zeros(2, 2);
    Cyawm_points = zeros(2, 2);
    
    % 격자점 데이터 추출
    for i = 1:2
        for j = 1:2
            if i == 1
                curr_alpha = alpha_low;
                alpha_idx = alpha_idx_low;
            else
                curr_alpha = alpha_high;
                alpha_idx = alpha_idx_high;
            end
            
            if j == 1
                curr_mach = mach_low;
                mach_idx = mach_idx_low;
            else
                curr_mach = mach_high;
                mach_idx = mach_idx_high;
            end
            
            % 해당 격자점 데이터 찾기
            mask = (aero_data.AoA == alpha_range(alpha_idx)) & ...
                   (aero_data.Mach == mach_range(mach_idx));
            
            if any(mask)
                row = aero_data(mask, :);
                CA_points(i, j) = row.CA(1);
                CY_points(i, j) = row.CY(1);
                CN_points(i, j) = row.CN(1);
                
                if ismember('Clm', row.Properties.VariableNames)
                    Clm_points(i, j) = row.Clm(1);
                end
                
                if ismember('Cmm', row.Properties.VariableNames)
                    Cmm_points(i, j) = row.Cmm(1);
                end
                
                if ismember('Cyawm', row.Properties.VariableNames)
                    Cyawm_points(i, j) = row.Cyawm(1);
                end
            end
        end
    end
    
    % 보간 가중치 계산
    if alpha_high ~= alpha_low
        alpha_weight = (alpha - alpha_low) / (alpha_high - alpha_low);
    else
        alpha_weight = 0;
    end
    
    if mach_high ~= mach_low
        mach_weight = (mach - mach_low) / (mach_high - mach_low);
    else
        mach_weight = 0;
    end
    
    % 바이리니어 보간 수행
    % 먼저 알파 방향으로 보간
    CA_temp1 = CA_points(1, 1) + alpha_weight * (CA_points(2, 1) - CA_points(1, 1));
    CA_temp2 = CA_points(1, 2) + alpha_weight * (CA_points(2, 2) - CA_points(1, 2));
    % 다음 마하수 방향으로 보간
    CAm = CA_temp1 + mach_weight * (CA_temp2 - CA_temp1);
    
    % 나머지 계수들에 대해서도 동일한 방식으로 보간
    CY_temp1 = CY_points(1, 1) + alpha_weight * (CY_points(2, 1) - CY_points(1, 1));
    CY_temp2 = CY_points(1, 2) + alpha_weight * (CY_points(2, 2) - CY_points(1, 2));
    CYm = CY_temp1 + mach_weight * (CY_temp2 - CY_temp1);
    
    CN_temp1 = CN_points(1, 1) + alpha_weight * (CN_points(2, 1) - CN_points(1, 1));
    CN_temp2 = CN_points(1, 2) + alpha_weight * (CN_points(2, 2) - CN_points(1, 2));
    CNm = CN_temp1 + mach_weight * (CN_temp2 - CN_temp1);
    
    Clm_temp1 = Clm_points(1, 1) + alpha_weight * (Clm_points(2, 1) - Clm_points(1, 1));
    Clm_temp2 = Clm_points(1, 2) + alpha_weight * (Clm_points(2, 2) - Clm_points(1, 2));
    Clm = Clm_temp1 + mach_weight * (Clm_temp2 - Clm_temp1);
    
    Cmm_temp1 = Cmm_points(1, 1) + alpha_weight * (Cmm_points(2, 1) - Cmm_points(1, 1));
    Cmm_temp2 = Cmm_points(1, 2) + alpha_weight * (Cmm_points(2, 2) - Cmm_points(1, 2));
    Cmm = Cmm_temp1 + mach_weight * (Cmm_temp2 - Cmm_temp1);
    
    Cyawm_temp1 = Cyawm_points(1, 1) + alpha_weight * (Cyawm_points(2, 1) - Cyawm_points(1, 1));
    Cyawm_temp2 = Cyawm_points(1, 2) + alpha_weight * (Cyawm_points(2, 2) - Cyawm_points(1, 2));
    Cyawm = Cyawm_temp1 + mach_weight * (Cyawm_temp2 - Cyawm_temp1);
    
    % NaN 값 체크 및 대체
    if isnan(CAm), CAm = 0.5; end
    if isnan(CYm), CYm = 0; end
    if isnan(CNm), CNm = 0; end
    if isnan(Clm), Clm = 0; end
    if isnan(Cmm), Cmm = 0; end
    if isnan(Cyawm), Cyawm = 0; end
end

% CSV 데이터에 기반한 정확한 속도 미분계수 계산 함수
function [CXu, CYv, CZw, Clp, Cmq, Cnr] = calcVelocityDerivatives_accurate(params, alpha_deg, mach, ALPHA_MESH, MACH_MESH, CA_mesh, CY_mesh, CN_mesh, Clpm, Cmqm, Cyawrm)
    % CSV 데이터에 기반한 정확한 속도 미분계수 계산 함수
    % 유한차분법과 2D 보간을 조합하여 미분계수를 계산
    
    % 환경 파라미터
    sound_speed = params.environment.sound_speed;
    
    % 미분 계산을 위한 마하수 변화량
    delta_mach = 0.01 * max(0.1, mach);  % 마하수의 1% 또는 최소 0.001
    
    % 마하수 기준 변화량
    mach_plus = min(mach + delta_mach, max(MACH_MESH(:)));
    mach_minus = max(mach - delta_mach, min(MACH_MESH(:)));
    
    % 공력 계수 보간 - 중앙 지점
    CA_center = interp2(ALPHA_MESH, MACH_MESH, CA_mesh, alpha_deg, mach, 'linear');
    CY_center = interp2(ALPHA_MESH, MACH_MESH, CY_mesh, alpha_deg, mach, 'linear');
    CN_center = interp2(ALPHA_MESH, MACH_MESH, CN_mesh, alpha_deg, mach, 'linear');
    
    % 마하수 +delta 지점에서의 공력 계수
    CA_plus = interp2(ALPHA_MESH, MACH_MESH, CA_mesh, alpha_deg, mach_plus, 'linear');
    CY_plus = interp2(ALPHA_MESH, MACH_MESH, CY_mesh, alpha_deg, mach_plus, 'linear');
    CN_plus = interp2(ALPHA_MESH, MACH_MESH, CN_mesh, alpha_deg, mach_plus, 'linear');
    
    % 마하수 -delta 지점에서의 공력 계수
    CA_minus = interp2(ALPHA_MESH, MACH_MESH, CA_mesh, alpha_deg, mach_minus, 'linear');
    CY_minus = interp2(ALPHA_MESH, MACH_MESH, CY_mesh, alpha_deg, mach_minus, 'linear');
    CN_minus = interp2(ALPHA_MESH, MACH_MESH, CN_mesh, alpha_deg, mach_minus, 'linear');
    
    % 중앙차분법으로 미분 계산 (dC/dM)
    dCA_dM = (CA_plus - CA_minus) / (mach_plus - mach_minus);
    dCY_dM = (CY_plus - CY_minus) / (mach_plus - mach_minus);
    dCN_dM = (CN_plus - CN_minus) / (mach_plus - mach_minus);
    
    % 속도 기준으로 변환 (dC/dV = dC/dM * dM/dV)
    CXu = dCA_dM / sound_speed;
    CYv = dCY_dM / sound_speed;
    CZw = dCN_dM / sound_speed;
    
    % 특별한 경우 처리 (마하수가 매우 낮은 경우나 보간 실패)
    if isnan(CXu) || isnan(CYv) || isnan(CZw) || mach < 0.05
        % 기본값 적용
        CXu = -0.02;
        CYv = -0.02;
        CZw = -0.02;
    end
    
    % 물리적 타당성 확인 (항력 계수는 일반적으로 음수)
    if CXu > 0
        CXu = -abs(CXu);
    end
    
    if CYv > 0
        CYv = -abs(CYv);
    end
    
    if CZw > 0
        CZw = -abs(CZw);
    end
    
    % 받음각에 따른 조정
    % 높은 받음각에서는 계수 강화
    if alpha_deg > 15
        alpha_factor = 1.0 + 0.05 * (alpha_deg - 15);
        alpha_factor = min(alpha_factor, 2.0);  % 최대 2배까지만 조정
        
        CXu = CXu * alpha_factor;
        CYv = CYv * alpha_factor;
        CZw = CZw * alpha_factor;
    end
    
    % 마하수에 따른 특성 조정
    % 천음속 영역에서 항력 변화가 급격함
    if mach > 0.8 && mach < 1.2
        trans_factor = 1.0 + 2.0 * (abs(mach - 1.0) / 0.2);
        trans_factor = min(trans_factor, 2.5);  % 최대 2.5배까지 조정
        
        CXu = CXu * trans_factor;
        CYv = CYv * trans_factor;
        CZw = CZw * trans_factor;
    end
    
    % 회전 속도 미분계수 (댐핑 계수)는 그대로 사용
    Clp = Clpm;
    Cmq = Cmqm;
    Cnr = Cyawrm;
    
    % 마하수 범위에 따른 회전 댐핑 계수 보정
    if mach > 1.0
        % 초음속 영역 - 댐핑 감소
        damping_scale = 0.8;
        Clp = Clp * damping_scale;
        Cmq = Cmq * damping_scale;
        Cnr = Cnr * damping_scale;
    elseif mach > 0.8
        % 천음속 영역 - 댐핑 변화
        trans_damp_factor = 0.7 + 0.6 * ((mach - 0.8) / 0.2);  % 0.7~1.3 범위
        Cmq = Cmq * trans_damp_factor;
        Cnr = Cnr * trans_damp_factor;
    end
end