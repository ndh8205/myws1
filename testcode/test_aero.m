clear all;
close all;
clc;

% 경로 설정
addpath(genpath('C:\Users\USER\Desktop\hanul_GNC'));

% 파일 경로 설정
force_moment_path = 'csv_Force_moment_arg.csv';  % 정적 공력계수 파일
damping_path = 'csv_damping.csv';     % 실제 감쇠 계수 파일
rkt_params_path = 'rocket.xml';       % 로켓 파라미터 파일
motor_csv_path = 'AeroTech_M2400T.csv'; % 모터 파일

% 1. CSV 파일 직접 로드 (열 이름 보존)
fprintf('CSV 데이터 직접 로드 중...\n');
try
    % 정적 공력계수 파일 읽기 - 열 이름 보존 설정
    opts = detectImportOptions(force_moment_path, 'VariableNamingRule', 'preserve');
    static_data = readtable(force_moment_path, opts);
    
    % 열 이름 출력
    fprintf('정적 계수 파일 열 이름: %s\n', strjoin(static_data.Properties.VariableNames, ', '));
    
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
    
    % 감쇠 공력계수 파일 읽기 - 열 이름 보존 설정
    opts = detectImportOptions(damping_path, 'VariableNamingRule', 'preserve');
    damping_data = readtable(damping_path, opts);
    
    fprintf('감쇠 계수 파일 로드 완료:\n');
    fprintf('  포함된 열: %s\n', strjoin(damping_data.Properties.VariableNames, ', '));
    fprintf('  감쇠 데이터 총 행 수: %d 행\n', height(damping_data));
    
    % 2. 데이터 영역 분석 - 마하수 0.83 기준으로 분리
    mach_threshold = 0.83;
    
    % 영역별 데이터 분리
    low_mach_idx = static_data.Mach <= mach_threshold;
    high_mach_idx = static_data.Mach > mach_threshold;
    
    % 각 영역별 데이터 개수 확인
    low_mach_data = static_data(low_mach_idx, :);
    high_mach_data = static_data(high_mach_idx, :);
    
    % 높은 마하수 영역의 받음각 확인
    high_mach_alphas = unique(high_mach_data.AoA);
    
    fprintf('\n데이터 영역 분석:\n');
    fprintf('  마하수 <= %.2f: %d개 행 (다양한 받음각)\n', mach_threshold, height(low_mach_data));
    fprintf('  마하수 > %.2f: %d개 행 (받음각 = ', mach_threshold, height(high_mach_data));
    fprintf('%.1f ', high_mach_alphas);
    fprintf(')\n');
    
    % 3. 데이터 가용성 맵 생성
    % 모든 (마하수, 받음각) 조합 생성
    [mach_grid, alpha_grid] = meshgrid(unique_mach, unique_alpha);
    data_exists = zeros(size(mach_grid));
    
    % 데이터 존재 여부 표시
    for i = 1:height(static_data)
        m_idx = find(unique_mach == static_data.Mach(i));
        a_idx = find(unique_alpha == static_data.AoA(i));
        data_exists(a_idx, m_idx) = 1;
    end
    
    % 전체 데이터 가용성 출력
    fprintf('  데이터 존재 비율: %.1f%% (%d/%d)\n', ...
            100*sum(data_exists(:))/numel(data_exists), ...
            sum(data_exists(:)), numel(data_exists));
    
catch ME
    fprintf('데이터 로드 실패: %s\n', ME.message);
    rethrow(ME);
end

% 로켓 파라미터 로드 (간략화)
fprintf('로켓 파라미터 로드 중...\n');
try
    params = struct();
    params.vehicle = struct();
    params.vehicle.D_ref = 0.136;  % 로켓 직경 [m]
    params.vehicle.S_A_ref = pi * (params.vehicle.D_ref/2)^2;  % 단면적 [m^2]
    params.environment.ro = 1.225;  % 대기 밀도 [kg/m^3]
    params.environment.sound_speed = 340;  % 음속 [m/s]
    fprintf('로켓 파라미터 기본값 설정 완료\n');
catch ME
    fprintf('로켓 파라미터 설정 실패: %s\n', ME.message);
end

% 4. 데이터 행렬 구성 - 주요 계수
% 주요 정적 계수 확인 및 이름 정규화
coef_names = {'CA', 'CY', 'CN', 'Clm', 'Cmm', 'Cyawm'};
actual_names = cell(size(coef_names));

% 실제 열 이름 확인 (대소문자 무시)
for i = 1:length(coef_names)
    found = false;
    for j = 1:length(static_data.Properties.VariableNames)
        if strcmpi(coef_names{i}, static_data.Properties.VariableNames{j})
            actual_names{i} = static_data.Properties.VariableNames{j};
            found = true;
            break;
        end
    end
    
    if ~found
        fprintf('경고: 계수 "%s"에 해당하는 열을 찾을 수 없습니다.\n', coef_names{i});
        actual_names{i} = '';
    end
end

% 유효한 계수만 추출
valid_coefs = ~cellfun(@isempty, actual_names);
coef_names = coef_names(valid_coefs);
actual_names = actual_names(valid_coefs);

fprintf('\n사용 가능한 정적 계수:\n');
for i = 1:length(coef_names)
    fprintf('  %s -> %s\n', coef_names{i}, actual_names{i});
end

% 5. 시각화

% 5.1 데이터 가용성 맵
figure('Name', '공력 데이터 가용성 맵');
imagesc(unique_mach, unique_alpha, data_exists);
colormap([1 1 1; 0 0.7 0]);  % 흰색(데이터 없음), 녹색(데이터 있음)
colorbar('Ticks', [0, 1], 'TickLabels', {'데이터 없음', '데이터 있음'});
xlabel('마하수');
ylabel('받음각 [\circ]');
title('정적 공력 계수 데이터 가용성');
grid on;

% X축 틱 라벨 설정
xticks(unique_mach);

% 마하수 기준 분리선 추가
hold on;
threshold_idx = find(unique_mach >= mach_threshold, 1);
if ~isempty(threshold_idx) && threshold_idx > 1
    x_threshold = (unique_mach(threshold_idx-1) + unique_mach(threshold_idx))/2;
    plot([x_threshold x_threshold], [min(unique_alpha)-0.5, max(unique_alpha)+0.5], 'r--', 'LineWidth', 2);
    text(x_threshold, max(unique_alpha)+1, sprintf('Ma = %.2f', mach_threshold), ...
         'Color', 'r', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
end
hold off;

% 5.2 영역별 정적 계수 시각화
for coef_idx = 1:length(coef_names)
    coef_name = coef_names{coef_idx};
    actual_name = actual_names{coef_idx};
    
    figure('Name', sprintf('%s 공력 계수 분석', coef_name));
    
    % 1. 낮은 마하수 영역: 2D 컬러맵
    subplot(2, 2, 1);
    
    % 낮은 마하수 영역 데이터 추출
    coef_data = NaN(length(unique_alpha), length(unique_mach));
    
    for i = 1:height(static_data)
        m_idx = find(unique_mach == static_data.Mach(i));
        a_idx = find(unique_alpha == static_data.AoA(i));
        if ~isempty(m_idx) && ~isempty(a_idx)
            coef_data(a_idx, m_idx) = static_data.(actual_name)(i);
        end
    end
    
    % 낮은 마하수 영역 시각화
    low_mach_indices = unique_mach <= mach_threshold;
    imagesc(unique_mach(low_mach_indices), unique_alpha, coef_data(:, low_mach_indices));
    colorbar;
    colormap(jet);
    xlabel('마하수 (Ma \leq 0.83)');
    ylabel('받음각 [\circ]');
    title(sprintf('%s - 마하수 ≤ %.2f 영역', coef_name, mach_threshold));
    
    % 2. 높은 마하수 영역: 제로 받음각의 마하수 변화
    subplot(2, 2, 2);
    
    % 높은 마하수 영역 데이터 준비
    high_mach_indices = unique_mach > mach_threshold;
    high_machs = unique_mach(high_mach_indices);
    
    % 받음각 0도의 데이터 추출
    zero_alpha_idx = find(unique_alpha == 0);
    if ~isempty(zero_alpha_idx)
        zero_alpha_data = coef_data(zero_alpha_idx, high_mach_indices);
        
        % 플롯
        if sum(~isnan(zero_alpha_data)) > 0
            plot(high_machs, zero_alpha_data, 'ro-', 'LineWidth', 2, 'MarkerSize', 8);
            grid on;
            xlabel('마하수 (Ma > 0.83)');
            ylabel(coef_name);
            title(sprintf('%s - 받음각 = 0°, 마하수 > %.2f', coef_name, mach_threshold));
        else
            text(0.5, 0.5, '데이터 없음', 'HorizontalAlignment', 'center', 'FontSize', 14);
            axis off;
        end
    else
        text(0.5, 0.5, '받음각 0° 데이터 없음', 'HorizontalAlignment', 'center', 'FontSize', 14);
        axis off;
    end
    
    % 3. 마하수별 받음각 변화 곡선 (낮은 마하수 영역)
    subplot(2, 2, 3);
    hold on;
    
    % 낮은 마하수 중 선택된 값들
    low_machs = unique_mach(low_mach_indices);
    num_curves = min(5, length(low_machs));
    selected_indices = round(linspace(1, length(low_machs), num_curves));
    selected_machs = low_machs(selected_indices);
    colors = jet(num_curves);
    
    % 각 마하수별 곡선 그리기
    for i = 1:num_curves
        mach_idx = find(unique_mach == selected_machs(i));
        if ~isempty(mach_idx)
            curve_data = coef_data(:, mach_idx);
            plot(unique_alpha, curve_data, 'o-', 'LineWidth', 2, 'Color', colors(i,:), ...
                 'DisplayName', sprintf('Ma = %.2f', selected_machs(i)));
        end
    end
    
    grid on;
    xlabel('받음각 [\circ]');
    ylabel(coef_name);
    title(sprintf('%s - 받음각 변화 (마하수 ≤ %.2f)', coef_name, mach_threshold));
    legend('Location', 'best');
    hold off;
    
    % 4. 트랜소닉 영역 확대 (마하수 0.7-1.2)
    subplot(2, 2, 4);
    
    % 트랜소닉 영역 범위
    transonic_min = 0.7;
    transonic_max = 1.2;
    transonic_indices = unique_mach >= transonic_min & unique_mach <= transonic_max;
    
    if sum(transonic_indices) > 0
        transonic_machs = unique_mach(transonic_indices);
        
        % 받음각 0도의 데이터 추출
        if ~isempty(zero_alpha_idx)
            transonic_data = coef_data(zero_alpha_idx, transonic_indices);
            
            % 플롯
            if sum(~isnan(transonic_data)) > 0
                plot(transonic_machs, transonic_data, 'bs-', 'LineWidth', 2, 'MarkerSize', 8);
                
                % 마하수 = 1 표시
                hold on;
                plot([1 1], get(gca, 'YLim'), 'k--', 'LineWidth', 1.5);
                text(1, mean(get(gca, 'YLim')), ' Ma = 1', 'FontWeight', 'bold');
                hold off;
                
                grid on;
                xlabel('마하수 (트랜소닉 영역)');
                ylabel(coef_name);
                title(sprintf('%s - 받음각 = 0°, 트랜소닉 영역', coef_name));
                xlim([transonic_min, transonic_max]);
            else
                text(0.5, 0.5, '데이터 없음', 'HorizontalAlignment', 'center', 'FontSize', 14);
                axis off;
            end
        else
            text(0.5, 0.5, '받음각 0° 데이터 없음', 'HorizontalAlignment', 'center', 'FontSize', 14);
            axis off;
        end
    else
        text(0.5, 0.5, '트랜소닉 영역 데이터 없음', 'HorizontalAlignment', 'center', 'FontSize', 14);
        axis off;
    end
    
    % 전체 제목
    sgtitle(sprintf('%s 공력 계수 영역별 분석', coef_name), 'FontSize', 16, 'FontWeight', 'bold');
end

% 5.3 영역 데이터 연결 및 데이터 결합 시각화
% - CN과 Cmm 계수만 특별히 더 자세하게 분석 (가장 중요한 계수들)
main_coefs = {'CN', 'Cmm'};
main_actual_names = cell(size(main_coefs));

% 실제 이름 찾기
for i = 1:length(main_coefs)
    idx = find(strcmpi(main_coefs{i}, coef_names));
    if ~isempty(idx)
        main_actual_names{i} = actual_names{idx};
    else
        main_actual_names{i} = '';
    end
end

% 유효한 계수만 사용
valid_main_coefs = ~cellfun(@isempty, main_actual_names);
main_coefs = main_coefs(valid_main_coefs);
main_actual_names = main_actual_names(valid_main_coefs);

% 각 주요 계수에 대해 3D 시각화
for i = 1:length(main_coefs)
    coef_name = main_coefs{i};
    actual_name = main_actual_names{i};
    
    figure('Name', sprintf('%s 3D 시각화', coef_name));
    
    % 모든 데이터 포인트 준비
    x_data = [];
    y_data = [];
    z_data = [];
    
    for j = 1:height(static_data)
        x_data(end+1) = static_data.AoA(j);
        y_data(end+1) = static_data.Mach(j);
        z_data(end+1) = static_data.(actual_name)(j);
    end
    
    % 3D 산점도
    scatter3(x_data, y_data, z_data, 50, z_data, 'filled');
    colorbar;
    colormap(jet);
    xlabel('받음각 [\circ]');
    ylabel('마하수');
    zlabel(coef_name);
    title(sprintf('%s 공력 계수 3D 시각화', coef_name));
    grid on;
    
    % 특정 받음각에서의 모든 마하수 데이터 연결
    hold on;
    for alpha_val = unique_alpha'
        alpha_indices = find(x_data == alpha_val);
        if length(alpha_indices) > 1
            [sorted_mach, sort_idx] = sort(y_data(alpha_indices));
            plot3(x_data(alpha_indices(sort_idx)), sorted_mach, z_data(alpha_indices(sort_idx)), ...
                  'k-', 'LineWidth', 1);
        end
    end
    
    % 가시성 향상을 위한 회전 설정
    view(30, 30);
    
    % 마하수 = 0.83 평면 추가
    xlims = get(gca, 'XLim');
    zlims = get(gca, 'ZLim');
    [X, Z] = meshgrid([xlims(1), xlims(2)], [zlims(1), zlims(2)]);
    Y = ones(size(X)) * mach_threshold;
    surf(X, Y, Z, 'FaceAlpha', 0.2, 'FaceColor', 'r', 'EdgeColor', 'r', 'DisplayName', sprintf('Ma = %.2f', mach_threshold));
    
    hold off;
end

% 6. 감쇠 계수 데이터 요약 및 시각화 (간략화)
damping_keys = {'Clpm', 'Cmqm', 'Cyawrm'};
valid_damping_keys = ismember(damping_keys, damping_data.Properties.VariableNames);

if sum(valid_damping_keys) > 0
    figure('Name', '감쇠 계수 요약');
    
    % 감쇠 계수 존재 여부에 따라 플롯 구성
    for idx = 1:sum(valid_damping_keys)
        key_idx = find(valid_damping_keys, idx);
        key = damping_keys{key_idx(end)};
        
        subplot(1, sum(valid_damping_keys), idx);
        
        % 감쇠 계수 0도 받음각 데이터만 추출 (마하수 변화에 따른)
        zero_deg_idx = damping_data.AoA == 0;
        if sum(zero_deg_idx) > 0
            zero_deg_data = damping_data(zero_deg_idx, :);
            
            % 마하수 증가 순으로 정렬
            [sorted_mach, sort_idx] = sort(zero_deg_data.Mach);
            sorted_data = zero_deg_data.(key)(sort_idx);
            
            % 플롯
            plot(sorted_mach, sorted_data, 'bo-', 'LineWidth', 2, 'MarkerSize', 6);
            grid on;
            xlabel('마하수');
            ylabel(key);
            title(sprintf('%s - 받음각 0° 기준', key));
            
            % 마하수=0.83 표시
            hold on;
            yrange = get(gca, 'YLim');
            plot([mach_threshold mach_threshold], yrange, 'r--', 'LineWidth', 1.5);
            text(mach_threshold, mean(yrange), sprintf(' Ma=%.2f', mach_threshold), 'Color', 'r');
            hold off;
        else
            text(0.5, 0.5, '받음각 0° 데이터 없음', 'HorizontalAlignment', 'center', 'FontSize', 14);
            axis off;
        end
    end
    
    sgtitle('감쇠 계수 마하수 변화 (0도 받음각)', 'FontSize', 14);
end

fprintf('시각화 완료! 총 %d개의 그래프가 생성되었습니다.\n', length(coef_names) + 2);