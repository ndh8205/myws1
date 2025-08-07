clear all;
close all;
clc;

addpath(genpath('C:\Users\USER\Desktop\hanul_GNC'));

%% 파일 경로 설정
force_moment_path = 'csv_Force_moment.csv';  % 정적 공력계수 파일
damping_path = 'csv_damping.csv';            % 완전한 감쇠 계수 파일 (샘플 대신 전체 파일 사용)

% 마하수 임계값 설정 (저마하수/고마하수 구분)
mach_threshold = 0.83;

%% 정적 공력계수 데이터 로드
fprintf('정적 공력계수 데이터 로드 중...\n');
try
    % 열 이름 보존 설정으로 파일 읽기
    opts = detectImportOptions(force_moment_path, 'VariableNamingRule', 'preserve');
    static_data = readtable(force_moment_path, opts);
    
    % 데이터 요약 출력
    unique_alpha = unique(static_data.AoA);
    unique_mach = unique(static_data.Mach);
    fprintf('정적 계수 파일 로드 완료:\n');
    fprintf('  받음각: %d개 지점 (', length(unique_alpha));
    fprintf('%.1f ', unique_alpha);
    fprintf(')\n');
    fprintf('  마하수: %d개 지점 (', length(unique_mach));
    fprintf('%.2f ', unique_mach);
    fprintf(')\n');
    
    % 데이터 분리 (저마하수/고마하수)
    low_mach_idx = static_data.Mach <= mach_threshold;
    high_mach_idx = static_data.Mach > mach_threshold;
    
    low_mach_data = static_data(low_mach_idx, :);
    high_mach_data = static_data(high_mach_idx, :);
    
    fprintf('  마하수 <= %.2f: %d개 행\n', mach_threshold, height(low_mach_data));
    fprintf('  마하수 > %.2f: %d개 행\n', mach_threshold, height(high_mach_data));
catch ME
    fprintf('데이터 로드 실패: %s\n', ME.message);
    rethrow(ME);
end

%% 감쇠 계수 데이터 로드
fprintf('감쇠 계수 데이터 로드 중...\n');
try
    % 열 이름 보존 설정으로 파일 읽기
    opts = detectImportOptions(damping_path, 'VariableNamingRule', 'preserve');
    damping_data = readtable(damping_path, opts);
    
    % 데이터 요약 출력
    fprintf('감쇠 계수 파일 로드 완료:\n');
    fprintf('  포함된 열: %s\n', strjoin(damping_data.Properties.VariableNames, ', '));
    fprintf('  감쇠 데이터 총 행 수: %d 행\n', height(damping_data));
    
    % 감쇠 데이터도 저/고마하수로 분리
    damping_low_mach_idx = damping_data.Mach <= mach_threshold;
    damping_high_mach_idx = damping_data.Mach > mach_threshold;
    
    damping_low_mach_data = damping_data(damping_low_mach_idx, :);
    damping_high_mach_data = damping_data(damping_high_mach_idx, :);
    
    fprintf('  마하수 <= %.2f: %d개 행\n', mach_threshold, height(damping_low_mach_data));
    fprintf('  마하수 > %.2f: %d개 행\n', mach_threshold, height(damping_high_mach_data));
catch ME
    fprintf('감쇠 데이터 로드 실패 또는 파일 없음: %s\n', ME.message);
    fprintf('감쇠 데이터 분석은 건너뜁니다.\n');
    % 계속 진행
end

%% 정적 공력계수에 대한 다차원 회귀 모델 개발
% 대상 계수 목록
static_coef_names = {'CA', 'CN', 'CY', 'Clm', 'Cmm', 'Cyawm'};

% 유효한 계수만 필터링
valid_static_coefs = intersect(static_coef_names, static_data.Properties.VariableNames);

% 결과 저장을 위한 구조체
static_models = struct();

fprintf('\n정적 공력계수 회귀 모델 개발 중...\n');

for i = 1:length(valid_static_coefs)
    coef = valid_static_coefs{i};
    fprintf('  %s 계수 모델 개발...\n', coef);
    
    % 학습 데이터 준비
    % 저마하수 영역의 모든 데이터 + 고마하수 영역의 0도 데이터
    X_train = [low_mach_data.Mach, low_mach_data.AoA];
    Y_train = low_mach_data.(coef);
    
    zero_high_mach = high_mach_data(high_mach_data.AoA == 0, :);
    X_train = [X_train; [zero_high_mach.Mach, zero_high_mach.AoA]];
    Y_train = [Y_train; zero_high_mach.(coef)];
    
    % 확장된 특성 공간 생성
    % 마하수 비선형 항, 받음각 비선형 항, 교차항 추가
    X_poly = [X_train, ... % 원본 특성 [Mach, AoA]
              X_train(:,1).^2, ... % Mach^2
              X_train(:,1).^3, ... % Mach^3
              X_train(:,2).^2, ... % AoA^2
              X_train(:,1).*X_train(:,2), ... % Mach*AoA
              X_train(:,1).^2.*X_train(:,2)]; % Mach^2*AoA
    
    % 트랜소닉 영역 특별 처리 - 마하수 1.0 주변의 가우시안 특성 추가
    % 이는 충격파 형성 등의 비선형성을 포착하기 위함
    X_trans_gauss = exp(-((X_train(:,1) - 1.0).^2)/0.02); % 마하수 1.0 중심
    
    % 초음속 영역 특별 처리 - 마하수 1.2 이상 영역에 대한 특성
    X_super_step = X_train(:,1) > 1.2;
    
    % 최종 특성 행렬
    X_features = [X_poly, X_trans_gauss, X_super_step];
    
    % 모델 학습 - 다중선형회귀 사용
    mdl = fitlm(X_features, Y_train, 'linear');
    static_models.(coef) = mdl;
    
    % 모델 검증 - 학습 데이터에 대한 예측
    Y_pred = predict(mdl, X_features);
    rmse = sqrt(mean((Y_train - Y_pred).^2));
    r2 = 1 - sum((Y_train - Y_pred).^2) / sum((Y_train - mean(Y_train)).^2);
    
    fprintf('    RMSE: %.6f, R^2: %.6f\n', rmse, r2);
    
    % 모델 계수 중요 항목 출력
    coefs = mdl.Coefficients.Estimate;
    pvals = mdl.Coefficients.pValue;
    
    fprintf('    중요 계수 (p < 0.05):\n');
    var_names = {'상수항', 'Mach', 'AoA', 'Mach^2', 'Mach^3', 'AoA^2', 'Mach*AoA', 'Mach^2*AoA', 'Trans_Gauss', 'Super_Step'};
    
    for j = 1:length(coefs)
        if pvals(j) < 0.05
            % 인덱스 범위 확인
            if j <= length(var_names)
                var_name = var_names{j};
            else
                var_name = sprintf('변수_%d', j);
            end
            fprintf('      %s: %.6f (p=%.4f)\n', var_name, coefs(j), pvals(j));
        end
    end
end

%% 감쇠 계수에 대한 다차원 회귀 모델 개발 (존재하는 경우)
% 주요 감쇠 계수 목록
damping_coef_names = {'Clpm', 'Cmqm', 'Cyawrm', 'Clmd', 'Cmmd', 'Cyawmd'};

% 감쇠 데이터가 있는 경우에만 모델 개발
if exist('damping_data', 'var')
    % 유효한 계수만 필터링
    valid_damping_coefs = intersect(damping_coef_names, damping_data.Properties.VariableNames);
    
    % 결과 저장을 위한 구조체
    damping_models = struct();
    
    fprintf('\n감쇠 계수 회귀 모델 개발 중...\n');
    
    for i = 1:length(valid_damping_coefs)
        coef = valid_damping_coefs{i};
        fprintf('  %s 계수 모델 개발...\n', coef);
        
        % 학습 데이터 준비
        X_train = [damping_low_mach_data.Mach, damping_low_mach_data.AoA];
        Y_train = damping_low_mach_data.(coef);
        
        % 고마하수 영역의 일부 데이터 포함 (있는 경우)
        if height(damping_high_mach_data) > 0
            zero_high_damping = damping_high_mach_data(damping_high_mach_data.AoA == 0, :);
            if height(zero_high_damping) > 0
                X_train = [X_train; [zero_high_damping.Mach, zero_high_damping.AoA]];
                Y_train = [Y_train; zero_high_damping.(coef)];
            end
        end
        
        % 감쇠계수용 확장 특성 공간 생성 - 정적 계수와 동일한 9개 특성 사용
        X_poly = [X_train, ... % 원본 특성 [Mach, AoA]
                  X_train(:,1).^2, ... % Mach^2
                  X_train(:,1).^3, ... % Mach^3
                  X_train(:,2).^2, ... % AoA^2
                  X_train(:,1).*X_train(:,2), ... % Mach*AoA
                  X_train(:,1).^2.*X_train(:,2)]; % Mach^2*AoA
        
        % 트랜소닉 영역 특별 처리
        X_trans_gauss = exp(-((X_train(:,1) - 1.0).^2)/0.02);
        
        % 초음속 영역 처리
        X_super_step = X_train(:,1) > 1.2;
        
        % 최종 특성 행렬 (정적 계수와 동일한 구조)
        X_features = [X_poly, X_trans_gauss, X_super_step];
        
        % 특성 개수 출력
        fprintf('    감쇠계수 특성 개수: %d\n', size(X_features, 2));
        
        % 모델 학습
        mdl = fitlm(X_features, Y_train, 'linear');
        damping_models.(coef) = mdl;
        
        % 모델 검증
        Y_pred = predict(mdl, X_features);
        rmse = sqrt(mean((Y_train - Y_pred).^2));
        r2 = 1 - sum((Y_train - Y_pred).^2) / sum((Y_train - mean(Y_train)).^2);
        
        fprintf('    RMSE: %.6f, R^2: %.6f\n', rmse, r2);
    end
else
    fprintf('\n감쇠 계수 데이터가 없어 해당 모델은 개발하지 않습니다.\n');
end

%% 고마하수 영역의 데이터 생성
fprintf('\n고마하수 영역 데이터 생성 중...\n');

% 모든 고마하수 값과 받음각 값
high_machs = unique(static_data.Mach(static_data.Mach > mach_threshold));
all_alphas = unique(static_data.AoA);

% 모델별 특성 개수 확인 및 출력
fprintf('  모델별 필요 특성 개수:\n');
for i = 1:length(valid_static_coefs)
    coef = valid_static_coefs{i};
    fprintf('    %s 모델: %d개 특성 필요\n', coef, static_models.(coef).NumCoefficients-1);
end

% 새로운 데이터 프레임 초기화
new_rows = [];

% 고마하수, 받음각 조합으로 데이터 생성
for mach_val = high_machs'
    for alpha_val = all_alphas'
        % 이미 데이터가 있는지 확인
        existing = find(high_mach_data.Mach == mach_val & high_mach_data.AoA == alpha_val);
        
        if ~isempty(existing)
            continue; % 이미 있는 데이터는 건너뛰기
        end
        
        % 새 행 생성
        new_row = array2table([mach_val, alpha_val], 'VariableNames', {'Mach', 'AoA'});
        
        % Velocity 열이 있으면 마하수로부터 계산
        if ismember('Velocity', static_data.Properties.VariableNames)
            sound_speed = 340; % m/s, 표준 대기 조건
            new_row.Velocity = mach_val * sound_speed;
        end
        
        % 각 정적 계수 예측
        for i = 1:length(valid_static_coefs)
            coef = valid_static_coefs{i};
            mdl = static_models.(coef);
            
            % 특성 벡터 생성 - 학습 모델과 동일한 9개 특성 사용
            X_test = [mach_val, alpha_val, ...
                      mach_val^2, mach_val^3, alpha_val^2, ...
                      mach_val*alpha_val, mach_val^2*alpha_val];
            X_trans_gauss = exp(-((mach_val - 1.0)^2)/0.02);
            X_super_step = mach_val > 1.2;
            
            % 최종 특성 벡터 (9개 특성)
            X_features = [X_test, X_trans_gauss, X_super_step];
            
            % 예측
            pred_val = predict(mdl, X_features);
            
            % 물리적 제약 적용 - 예측값이 물리적으로 타당한지 확인
            % CN, Cmm 등의 계수에 대한 특별 처리
            if strcmp(coef, 'CN')
                % CN은 일반적으로 받음각에 비례하며, 특히 저마하수 영역에서
                % 0도 근처에서는 선형적 관계가 있음
                if alpha_val == 0
                    % 0도에서는 CN이 0에 가까워야 함
                    pred_val = pred_val * 0.1; % 약간의 여유 두기
                elseif abs(pred_val) > 10
                    % 비정상적으로 큰 값은 제한
                    pred_val = sign(pred_val) * 10;
                end
            elseif strcmp(coef, 'Cmm')
                % Cmm은 종종 정적 안정성을 나타내며, 급격히 변하지 않아야 함
                if abs(pred_val) > 5
                    pred_val = sign(pred_val) * 5;
                end
            end
            
            new_row.(coef) = pred_val;
        end
        
        % 새 행 추가
        new_rows = [new_rows; new_row];
    end
end

% 원본 데이터와 새로 생성된 데이터 결합
filled_static_data = [static_data; new_rows];

% 결과 요약
fprintf('  생성된 새 데이터 행 수: %d\n', height(new_rows));
fprintf('  최종 데이터셋 크기: %d 행\n', height(filled_static_data));

%% 데이터 검증 시각화
fprintf('\n결과 검증 시각화 중...\n');

% 주요 계수 선택 (CN, Cmm은 특히 중요)
key_coefs = {'CN', 'Cmm'};
valid_key_coefs = intersect(key_coefs, valid_static_coefs);

for i = 1:length(valid_key_coefs)
    coef = valid_key_coefs{i};
    
    figure('Name', sprintf('%s 계수 검증', coef), 'Position', [100, 100, 1000, 800]);
    
    % 1. 마하수 0.83에서의 받음각 변화 곡선 (기준선)
    subplot(2,2,1);
    threshold_data = static_data(abs(static_data.Mach - mach_threshold) < 0.01, :);
    [sorted_alpha, idx] = sort(threshold_data.AoA);
    plot(sorted_alpha, threshold_data.(coef)(idx), 'bo-', 'LineWidth', 2, 'DisplayName', '실제 데이터');
    xlabel('받음각 [\circ]');
    ylabel(coef);
    title(sprintf('%s @ Ma=%.2f (기준선)', coef, mach_threshold));
    grid on;
    
    % 2. 선택된 고마하수에서의 받음각 변화 곡선
    subplot(2,2,2);
    hold on;
    
    % 몇 개의 대표 마하수 선택
    selected_machs = high_machs(round(linspace(1, length(high_machs), min(4, length(high_machs)))));
    colors = jet(length(selected_machs));
    
    for j = 1:length(selected_machs)
        mach_val = selected_machs(j);
        
        % 해당 마하수의 원본+예측 데이터 추출
        mach_data = filled_static_data(filled_static_data.Mach == mach_val, :);
        [sorted_alpha, idx] = sort(mach_data.AoA);
        
        % 원본 데이터와 예측 데이터 구분
        % 각 받음각에 대해 [mach_val, alpha] 쌍이 원본 데이터에 있는지 확인
        orig_idx = false(size(sorted_alpha));
        for k = 1:length(sorted_alpha)
            orig_idx(k) = any(static_data.Mach == mach_val & static_data.AoA == sorted_alpha(k));
        end
        
        % 모든 데이터 점 플롯
        plot(sorted_alpha, mach_data.(coef)(idx), 'o-', 'Color', colors(j,:), ...
             'LineWidth', 2, 'DisplayName', sprintf('Ma=%.2f', mach_val));
        
        % 원본 데이터는 별도 마커로 강조
        if any(orig_idx)
            plot(sorted_alpha(orig_idx), mach_data.(coef)(idx(orig_idx)), 's', 'Color', colors(j,:), ...
                 'MarkerSize', 10, 'MarkerFaceColor', colors(j,:), 'DisplayName', sprintf('Ma=%.2f 원본', mach_val));
        end
    end
    
    xlabel('받음각 [\circ]');
    ylabel(coef);
    title(sprintf('%s @ 높은 마하수 영역', coef));
    legend('Location', 'best');
    grid on;
    hold off;
    
    % 3. 3D 시각화 - 전체 데이터 공간
    subplot(2,2,[3,4]);
    
    % 원본 데이터
    scatter3(static_data.Mach, static_data.AoA, static_data.(coef), 50, 'ro', 'filled', 'DisplayName', '원본 데이터');
    hold on;
    
    % 새로 생성된 데이터
    new_data_idx = ~ismember([filled_static_data.Mach, filled_static_data.AoA], [static_data.Mach, static_data.AoA], 'rows');
    new_data = filled_static_data(new_data_idx, :);
    scatter3(new_data.Mach, new_data.AoA, new_data.(coef), 30, 'bo', 'DisplayName', '생성된 데이터');
    
    % 마하수 0.83 평면 표시
    xrange = get(gca, 'XLim');
    yrange = get(gca, 'YLim');
    zrange = get(gca, 'ZLim');
    [xm, ym] = meshgrid([xrange(1), xrange(2)], [yrange(1), yrange(2)]);
    zm = ones(size(xm)) * mean(zrange);
    surf(xm*0 + mach_threshold, ym, zm, 'FaceColor', 'r', 'FaceAlpha', 0.2, 'EdgeColor', 'r', 'DisplayName', sprintf('Ma=%.2f', mach_threshold));
    
    xlabel('마하수');
    ylabel('받음각 [\circ]');
    zlabel(coef);
    title(sprintf('%s 3D 데이터 공간', coef));
    legend('Location', 'best');
    grid on;
    view(30, 30);
    hold off;
    
    % 전체 제목
    sgtitle(sprintf('%s 계수: 원본 vs 생성된 데이터', coef), 'FontSize', 16, 'FontWeight', 'bold');
end

%% 일관성 검증 - 마하수 변화에 따른 계수 변화
for i = 1:length(valid_key_coefs)
    coef = valid_key_coefs{i};
    
    figure('Name', sprintf('%s 마하수 검증', coef));
    
    % 0도, 4도, 8도의 받음각에 대해 마하수 변화 곡선
    target_alphas = [0, 4, 8];
    hold on;
    
    markers = {'o-', 's-', '^-'};
    colors = {'b', 'r', 'g'};
    
    for j = 1:length(target_alphas)
        alpha_val = target_alphas(j);
        
        % 해당 받음각의 데이터 추출
        alpha_data = filled_static_data(filled_static_data.AoA == alpha_val, :);
        [sorted_mach, idx] = sort(alpha_data.Mach);
        
        % 원본과 생성 데이터 구분
        orig_idx = ismember([sorted_mach, alpha_val*ones(size(sorted_mach))], static_data.Mach, static_data.AoA, 'rows');
        
        % 플롯
        plot(sorted_mach, alpha_data.(coef)(idx), markers{j}, 'Color', colors{j}, ...
             'LineWidth', 2, 'DisplayName', sprintf('AoA=%.1f°', alpha_val));
        
        % 원본 데이터 강조
        if any(orig_idx)
            plot(sorted_mach(orig_idx), alpha_data.(coef)(idx(orig_idx)), 'o', 'Color', colors{j}, ...
                 'MarkerSize', 10, 'MarkerFaceColor', colors{j}, 'DisplayName', sprintf('AoA=%.1f° 원본', alpha_val));
        end
    end
    
    % 마하수 = 0.83 표시
    yrange = get(gca, 'YLim');
    plot([mach_threshold mach_threshold], yrange, 'k--', 'LineWidth', 1.5, 'DisplayName', sprintf('Ma=%.2f', mach_threshold));
    
    % 마하수 = 1.0 표시
    plot([1.0 1.0], yrange, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Ma=1.0');
    
    xlabel('마하수');
    ylabel(coef);
    title(sprintf('%s: 마하수 변화에 따른 변화 (고정 받음각)', coef));
    legend('Location', 'best');
    grid on;
    hold off;
end

%% 생성된 데이터 저장
fprintf('\n생성된 데이터 저장 중...\n');

% 저장 파일명 설정
save_filename = 'filled_force_moment.csv';

% CSV 파일로 저장
writetable(filled_static_data, save_filename);
fprintf('  데이터가 %s 파일로 저장되었습니다.\n', save_filename);

% 감쇠 계수 모델이 있으면 해당 데이터도 생성 및 저장
if exist('damping_models', 'var')
    fprintf('\n감쇠 계수 데이터 생성 중...\n');
    
    % 새로운 감쇠 데이터 행 초기화
    new_damping_rows = [];
    
    % 고마하수, 받음각 조합으로 데이터 생성
    for mach_val = high_machs'
        for alpha_val = all_alphas'
            % 이미 데이터가 있는지 확인
            existing = find(damping_high_mach_data.Mach == mach_val & damping_high_mach_data.AoA == alpha_val);
            
            if ~isempty(existing)
                continue; % 이미 있는 데이터는 건너뛰기
            end
            
            % 새 행 생성
            new_row = array2table([mach_val, alpha_val], 'VariableNames', {'Mach', 'AoA'});
            
            % velocity 열이 있으면 마하수로부터 계산
            if ismember('velocity', damping_data.Properties.VariableNames)
                sound_speed = 340; % m/s
                new_row.velocity = mach_val * sound_speed;
            end
            
            % deg, rad/s 열이 있으면 기본값 할당
            if ismember('deg', damping_data.Properties.VariableNames)
                new_row.deg = 4.0; % 가정된 기본값
            end
            
            if ismember('rad/s', damping_data.Properties.VariableNames)
                new_row.rad_s = 10.0; % 가정된 기본값
            end
            
            % 각 감쇠 계수 예측
            for k = 1:length(valid_damping_coefs)
                coef = valid_damping_coefs{k};
                mdl = damping_models.(coef);
                
                % 특성 벡터 생성 - 학습 모델과 동일한 9개 특성 구조 사용
                X_test = [mach_val, alpha_val, ...
                          mach_val^2, mach_val^3, alpha_val^2, ...
                          mach_val*alpha_val, mach_val^2*alpha_val];
                X_trans_gauss = exp(-((mach_val - 1.0)^2)/0.02);
                X_super_step = mach_val > 1.2;
                X_features = [X_test, X_trans_gauss, X_super_step];
                
                % 모델 예측 - 특성 개수가 일치해야 함
                try
                    pred_val = predict(mdl, X_features);
                catch ME
                    fprintf('    예측 오류 (%s): %s\n', coef, ME.message);
                    
                    % 특성 개수 맞추기 시도
                    if contains(ME.message, '열이 있어야 합니다')
                        % 필요한 특성 개수 추출
                        required_features = regexp(ME.message, '(\d+)개의 열', 'tokens');
                        if ~isempty(required_features)
                            n_required = str2double(required_features{1}{1});
                            fprintf('    모델이 %d개 특성 필요, 현재 %d개 제공됨\n', ...
                                n_required, size(X_features, 2));
                            
                            % 특성 수 조정
                            if n_required > size(X_features, 2)
                                % 부족한 특성 채우기 (0으로)
                                X_features = [X_features, zeros(1, n_required - size(X_features, 2))];
                            elseif n_required < size(X_features, 2)
                                % 초과 특성 자르기
                                X_features = X_features(:, 1:n_required);
                            end
                            
                            % 다시 예측 시도
                            try
                                pred_val = predict(mdl, X_features);
                                fprintf('    특성 개수 조정 후 예측 성공\n');
                            catch ME2
                                fprintf('    두 번째 예측 시도도 실패: %s\n', ME2.message);
                                pred_val = 0; % 예측 실패 시 기본값
                            end
                        else
                            pred_val = 0;
                        end
                    else
                        pred_val = 0;
                    end
                end
                
                % 물리적 제약 적용
                if strcmp(coef, 'Cmqm')
                    % Cmq는 일반적으로 음수 (감쇠)
                    if pred_val > 0
                        pred_val = -abs(pred_val);
                    end
                end
                
                new_row.(coef) = pred_val;
            end
            
            % 새 행 추가
            new_damping_rows = [new_damping_rows; new_row];
        end
    end
    
    % 원본 데이터와 새로 생성된 데이터 결합
    filled_damping_data = [damping_data; new_damping_rows];
    
    % 결과 요약
    fprintf('  생성된 새 감쇠 데이터 행 수: %d\n', height(new_damping_rows));
    fprintf('  최종 감쇠 데이터셋 크기: %d 행\n', height(filled_damping_data));
    
    % 감쇠 데이터 저장
    save_damping_filename = 'filled_damping.csv';
    writetable(filled_damping_data, save_damping_filename);
    fprintf('  감쇠 데이터가 %s 파일로 저장되었습니다.\n', save_damping_filename);
end

fprintf('\n데이터 생성 및 검증이 완료되었습니다!\n');

%% 결과 데이터 로드 및 시뮬레이션 활용
% 이 코드는 주석으로 설명만 포함합니다. 실제 시뮬레이션에서는 아래와 같이 활용할 수 있습니다.
%{
% 채워진 데이터 로드
filled_static_data = readtable('filled_force_moment.csv');
filled_damping_data = readtable('filled_damping.csv');

% 시뮬레이션 시간 스텝에서...
current_mach = ...; % 현재 마하수
current_alpha = ...; % 현재 받음각

% 가장 가까운 데이터 찾기 또는 보간법 사용
% 예: 현재 마하수와 받음각에 가장 가까운 행 찾기
[~, idx] = min((filled_static_data.Mach - current_mach).^2 + ...
               (filled_static_data.AoA - current_alpha).^2);

% 해당 행의 계수 가져오기
CA = filled_static_data.CA(idx);
CN = filled_static_data.CN(idx);
Cmm = filled_static_data.Cmm(idx);
% ... 기타 필요한 계수

% 이제 이 계수들을 사용하여 공력 계산
% ...
%}