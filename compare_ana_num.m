%% HANul 로켓 선형화 방법 비교: 심볼릭 vs 수치적
% Last update: 2025-05-06

%% 초기화
clc;
clear all;
close all;

% 라이브러리 경로 추가 (필요시 경로 수정)
addpath(genpath('C:\Users\USER\Desktop\hanul_GNC'));

% 파라미터 로드
xmlFilePath = 'rocket.xml';
rocketParams = extractRocketParams(xmlFilePath);
flatRocketParams = flattenRocketStructure(rocketParams);
motorCsvPath = 'AeroTech_M2400T.csv';
force_moment_path = 'csv_Force_moment_axiseq.csv';

% 초기 파라미터 설정
params = vehicle_params(flatRocketParams, 0, motorCsvPath, true);

%% 초기 상태 설정
pos = [0, 0, 0]';                           % 위치 [m] - 관성 좌표계
V_B = [0, 0, 0]';                           % 속도 [m/s] - 동체 좌표계
att_euler = [deg2rad(50), deg2rad(87), deg2rad(0)]; % 자세각 [rad]
att_quat = GetQUAT(att_euler(3), att_euler(2), att_euler(1))'; % 쿼터니언
omega = [0, 0, 0]';                          % 각속도 [rad/s]

X = [pos; V_B; att_quat; omega];             % 상태 벡터

% 제어 입력 설정 (각 카나드 각도 및 RCS 상태)
U = [deg2rad(5); deg2rad(3); deg2rad(-5); deg2rad(-3); 0; 1; 0; 1];

% 시간 스텝
dt = 1/400;  % 400Hz 샘플링

%% 테스트 파라미터 설정
num_test_points = 100;  % 테스트 포인트 수
random_scale = 0.1;     % 랜덤 변동 스케일

% 저장 배열 초기화
computation_time_sym = zeros(num_test_points, 1);
computation_time_num = zeros(num_test_points, 1);
cond_num_sym = zeros(num_test_points, 1);
cond_num_num = zeros(num_test_points, 1);
max_eig_sym = zeros(num_test_points, 1);
max_eig_num = zeros(num_test_points, 1);
matrix_diff_norm = zeros(num_test_points, 1);
prediction_error_sym = zeros(num_test_points, 1);
prediction_error_num = zeros(num_test_points, 1);

A_sym_list = cell(num_test_points, 1);
B_sym_list = cell(num_test_points, 1);
A_num_list = cell(num_test_points, 1);
B_num_list = cell(num_test_points, 1);

% 진행 상태 표시
hWait = waitbar(0, '선형화 방법 비교 중...', 'Name', '진행 상태');

%% 테스트 포인트 생성 및 평가
for i = 1:num_test_points
    % 1. 테스트 상태 생성 (약간의 랜덤 변동 추가)
    X_test = X + random_scale * randn(size(X)) .* [ones(3,1); ones(3,1); 0.01*ones(4,1); 0.1*ones(3,1)];
    
    % 쿼터니언 정규화
    X_test(7:10) = X_test(7:10) / norm(X_test(7:10));
    
    % 테스트 제어 입력 (약간의 랜덤 변동 추가)
    U_test = U + random_scale * randn(size(U)) .* [0.1*ones(4,1); ones(4,1)];
    
    % 2. 비선형 시스템 시뮬레이션 (참값 계산)
    [X_next_nonlin, ~, ~, ~, ~, ~, ~] = srk4(@vehicle_dynamics_HANul, X_test, U_test, zeros(6,1), dt, params, 0, dt);
    
    % 3. 심볼릭 선형화 수행 및 시간 측정
    tic;
    [A_sym, B_sym] = linearize_HANul_ana(X_test, U_test, params, dt);
    computation_time_sym(i) = toc;
    
    % 심볼릭 선형화 A, B 행렬 저장
    A_sym_list{i} = A_sym;
    B_sym_list{i} = B_sym;
    
    % 심볼릭 선형화 모델 예측
    quat_omega_indices = 7:13;
    x_quat_omega = X_test(quat_omega_indices);
    X_next_sym_pred_quat_omega = A_sym * x_quat_omega + B_sym * U_test;
    
    % 심볼릭 선형화 모델 행렬 조건수 계산
    try
        cond_num_sym(i) = cond(A_sym);
    catch
        cond_num_sym(i) = Inf;
    end
    
    % 심볼릭 선형화 모델 고유값 계산
    try
        eig_sym = eig(A_sym);
        max_eig_sym(i) = max(real(eig_sym));
    catch
        max_eig_sym(i) = NaN;
    end
    
    % 4. 수치적 선형화 수행 및 시간 측정
    tic;
    [A_num, B_num] = linearize_HANul_numerical(X_test, U_test, params, dt);
    computation_time_num(i) = toc;
    
    % 수치적 선형화 A, B 행렬 저장
    A_num_list{i} = A_num;
    B_num_list{i} = B_num;
    
    % 수치적 선형화 모델 예측
    X_next_num_pred_quat_omega = A_num * x_quat_omega + B_num * U_test;
    
    % 수치적 선형화 모델 행렬 조건수 계산
    try
        cond_num_num(i) = cond(A_num);
    catch
        cond_num_num(i) = Inf;
    end
    
    % 수치적 선형화 모델 고유값 계산
    try
        eig_num = eig(A_num);
        max_eig_num(i) = max(real(eig_num));
    catch
        max_eig_num(i) = NaN;
    end
    
    % 5. 행렬 차이 계산
    matrix_diff_norm(i) = norm([A_sym - A_num, B_sym - B_num], 'fro');
    
    % 6. 예측 정확도 계산
    X_next_nonlin_quat_omega = X_next_nonlin(quat_omega_indices);
    prediction_error_sym(i) = norm(X_next_nonlin_quat_omega - X_next_sym_pred_quat_omega);
    prediction_error_num(i) = norm(X_next_nonlin_quat_omega - X_next_num_pred_quat_omega);
    
    % 진행 상태 업데이트
    waitbar(i/num_test_points, hWait, sprintf('진행 중: %.1f%%', i/num_test_points*100));
end

% 진행 상태 창 닫기
close(hWait);

%% 결과 시각화 및 분석

% 1. 계산 시간 비교
figure('Name', '계산 시간 비교', 'NumberTitle', 'off', 'Position', [100, 100, 800, 600]);
subplot(2,1,1);
bar([mean(computation_time_sym), mean(computation_time_num)]);
set(gca, 'XTickLabel', {'심볼릭 선형화', '수치적 선형화'});
title('평균 계산 시간');
ylabel('시간 [s]');
grid on;

subplot(2,1,2);
boxplot([computation_time_sym, computation_time_num], 'Labels', {'심볼릭 선형화', '수치적 선형화'});
title('계산 시간 분포');
ylabel('시간 [s]');
grid on;

% 계산 시간 통계 출력
fprintf('\n===== 계산 시간 분석 =====\n');
fprintf('심볼릭 선형화 평균 시간: %.6f 초\n', mean(computation_time_sym));
fprintf('수치적 선형화 평균 시간: %.6f 초\n', mean(computation_time_num));
fprintf('심볼릭/수치적 시간 비율: %.2f 배\n', mean(computation_time_sym)/mean(computation_time_num));

% 2. 행렬 조건수 비교
figure('Name', '행렬 조건수 비교', 'NumberTitle', 'off', 'Position', [100, 100, 800, 600]);
subplot(2,1,1);
semilogy([mean(cond_num_sym(~isinf(cond_num_sym))), mean(cond_num_num(~isinf(cond_num_num)))]);
set(gca, 'XTickLabel', {'심볼릭 선형화', '수치적 선형화'});
title('평균 A 행렬 조건수 (로그 스케일)');
ylabel('조건수');
grid on;

subplot(2,1,2);
boxplot(log10([cond_num_sym(~isinf(cond_num_sym)), cond_num_num(~isinf(cond_num_num))]), 'Labels', {'심볼릭 선형화', '수치적 선형화'});
title('A 행렬 조건수 분포 (log10)');
ylabel('log10(조건수)');
grid on;

% 조건수 통계 출력
fprintf('\n===== 행렬 조건수 분석 =====\n');
fprintf('심볼릭 선형화 평균 조건수: %.2e\n', mean(cond_num_sym(~isinf(cond_num_sym))));
fprintf('수치적 선형화 평균 조건수: %.2e\n', mean(cond_num_num(~isinf(cond_num_num))));
fprintf('조건수 비율 (심볼릭/수치적): %.2f 배\n', mean(cond_num_sym(~isinf(cond_num_sym)))/mean(cond_num_num(~isinf(cond_num_num))));

% 3. 고유값 분석
figure('Name', '고유값 분석', 'NumberTitle', 'off', 'Position', [100, 100, 800, 600]);
subplot(2,1,1);
plot(1:num_test_points, max_eig_sym, 'b.-', 1:num_test_points, max_eig_num, 'r.-');
title('최대 고유값 실수부 비교');
xlabel('테스트 포인트 인덱스');
ylabel('최대 고유값 실수부');
legend('심볼릭 선형화', '수치적 선형화');
grid on;
yline(0, 'k--');  % 안정성 경계

subplot(2,1,2);
boxplot([max_eig_sym, max_eig_num], 'Labels', {'심볼릭 선형화', '수치적 선형화'});
title('최대 고유값 실수부 분포');
ylabel('최대 고유값 실수부');
grid on;
yline(0, 'k--');  % 안정성 경계

% 안정성 분석 통계
fprintf('\n===== 고유값 안정성 분석 =====\n');
fprintf('심볼릭 선형화 불안정 포인트 비율: %.2f%%\n', 100*sum(max_eig_sym > 0)/sum(~isnan(max_eig_sym)));
fprintf('수치적 선형화 불안정 포인트 비율: %.2f%%\n', 100*sum(max_eig_num > 0)/sum(~isnan(max_eig_num)));
fprintf('심볼릭 선형화 평균 최대 고유값: %.4f\n', mean(max_eig_sym(~isnan(max_eig_sym))));
fprintf('수치적 선형화 평균 최대 고유값: %.4f\n', mean(max_eig_num(~isnan(max_eig_num))));

% 4. 행렬 차이 분석
figure('Name', '행렬 차이 분석', 'NumberTitle', 'off', 'Position', [100, 100, 800, 300]);
plot(1:num_test_points, matrix_diff_norm, '.-');
title('심볼릭과 수치적 선형화 행렬 차이 (프로베니우스 노름)');
xlabel('테스트 포인트 인덱스');
ylabel('행렬 차이 노름');
grid on;

% 행렬 차이 통계
fprintf('\n===== 행렬 차이 분석 =====\n');
fprintf('평균 행렬 차이 (프로베니우스 노름): %.4e\n', mean(matrix_diff_norm));
fprintf('최대 행렬 차이 (프로베니우스 노름): %.4e\n', max(matrix_diff_norm));
fprintf('최소 행렬 차이 (프로베니우스 노름): %.4e\n', min(matrix_diff_norm));

% 5. 예측 정확도 비교
figure('Name', '예측 정확도 비교', 'NumberTitle', 'off', 'Position', [100, 100, 800, 600]);
subplot(2,1,1);
plot(1:num_test_points, prediction_error_sym, 'b.-', 1:num_test_points, prediction_error_num, 'r.-');
title('예측 오차 비교');
xlabel('테스트 포인트 인덱스');
ylabel('예측 오차 (L2 노름)');
legend('심볼릭 선형화', '수치적 선형화');
grid on;

subplot(2,1,2);
boxplot([prediction_error_sym, prediction_error_num], 'Labels', {'심볼릭 선형화', '수치적 선형화'});
title('예측 오차 분포');
ylabel('예측 오차 (L2 노름)');
grid on;

% 예측 오차 통계
fprintf('\n===== 예측 정확도 분석 =====\n');
fprintf('심볼릭 선형화 평균 예측 오차: %.4e\n', mean(prediction_error_sym));
fprintf('수치적 선형화 평균 예측 오차: %.4e\n', mean(prediction_error_num));
fprintf('심볼릭/수치적 예측 오차 비율: %.2f\n', mean(prediction_error_sym)/mean(prediction_error_num));

% 6. 세부 비교를 위한 특정 테스트 포인트 분석
% 가장 큰 차이를 보이는 테스트 포인트 선택
[~, worst_idx] = max(matrix_diff_norm);

% A 행렬 값 분포 비교
figure('Name', '행렬 요소 분포 비교', 'NumberTitle', 'off', 'Position', [100, 100, 1000, 600]);
subplot(2,2,1);
imagesc(A_sym_list{worst_idx});
title('심볼릭 A 행렬 (최대 차이 테스트 포인트)');
colorbar;
axis square;

subplot(2,2,2);
imagesc(A_num_list{worst_idx});
title('수치적 A 행렬 (최대 차이 테스트 포인트)');
colorbar;
axis square;

subplot(2,2,3);
diff_A = A_sym_list{worst_idx} - A_num_list{worst_idx};
imagesc(diff_A);
title('A 행렬 차이');
colorbar;
axis square;

subplot(2,2,4);
diff_B = B_sym_list{worst_idx} - B_num_list{worst_idx};
imagesc(diff_B);
title('B 행렬 차이');
colorbar;
axis square;

% 7. 요약 보고서
fprintf('\n===== 선형화 방법 비교 요약 =====\n');
fprintf('1. 계산 효율성: ');
if mean(computation_time_sym) < mean(computation_time_num)
    fprintf('심볼릭 선형화가 %.2f배 더 빠름\n', mean(computation_time_num)/mean(computation_time_sym));
else
    fprintf('수치적 선형화가 %.2f배 더 빠름\n', mean(computation_time_sym)/mean(computation_time_num));
end

fprintf('2. 수치 안정성: ');
if mean(cond_num_sym(~isinf(cond_num_sym))) < mean(cond_num_num(~isinf(cond_num_num)))
    fprintf('심볼릭 선형화가 %.2e배 더 안정적인 조건수\n', mean(cond_num_num(~isinf(cond_num_num)))/mean(cond_num_sym(~isinf(cond_num_sym))));
else
    fprintf('수치적 선형화가 %.2e배 더 안정적인 조건수\n', mean(cond_num_sym(~isinf(cond_num_sym)))/mean(cond_num_num(~isinf(cond_num_num))));
end

fprintf('3. 시스템 안정성: ');
if mean(max_eig_sym(~isnan(max_eig_sym))) < mean(max_eig_num(~isnan(max_eig_num)))
    fprintf('심볼릭 선형화가 %.4f만큼 더 안정적인 고유값\n', mean(max_eig_num(~isnan(max_eig_num))) - mean(max_eig_sym(~isnan(max_eig_sym))));
else
    fprintf('수치적 선형화가 %.4f만큼 더 안정적인 고유값\n', mean(max_eig_sym(~isnan(max_eig_sym))) - mean(max_eig_num(~isnan(max_eig_num))));
end

fprintf('4. 예측 정확도: ');
if mean(prediction_error_sym) < mean(prediction_error_num)
    fprintf('심볼릭 선형화가 %.2f배 더 정확함\n', mean(prediction_error_num)/mean(prediction_error_sym));
else
    fprintf('수치적 선형화가 %.2f배 더 정확함\n', mean(prediction_error_sym)/mean(prediction_error_num));
end

% 추천 선형화 방법
fprintf('\n>>>>>> 추천 선형화 방법: ');
% 각 기준에 가중치 부여하여 최종 점수 계산
score_sym = 0;
score_num = 0;

% 계산 효율성 (30%)
if mean(computation_time_sym) < mean(computation_time_num)
    score_sym = score_sym + 0.3;
else
    score_num = score_num + 0.3;
end

% 수치 안정성 (30%)
if mean(cond_num_sym(~isinf(cond_num_sym))) < mean(cond_num_num(~isinf(cond_num_num)))
    score_sym = score_sym + 0.3;
else
    score_num = score_num + 0.3;
end

% 예측 정확도 (40%)
if mean(prediction_error_sym) < mean(prediction_error_num)
    score_sym = score_sym + 0.4;
else
    score_num = score_num + 0.4;
end

if score_sym > score_num
    fprintf('심볼릭 선형화 (점수: %.2f vs %.2f)\n', score_sym, score_num);
else
    fprintf('수치적 선형화 (점수: %.2f vs %.2f)\n', score_num, score_sym);
end

fprintf('===================================\n');

%% 선형화 방법 개선 제안
fprintf('\n===== 선형화 방법 개선 제안 =====\n');
if mean(cond_num_sym(~isinf(cond_num_sym))) > 1e12
    fprintf('1. 심볼릭 선형화의 조건수가 매우 높습니다 (%.2e). 다음을 고려하세요:\n', mean(cond_num_sym(~isinf(cond_num_sym))));
    fprintf('   - 자코비안 계산 중 정밀도 문제가 발생할 수 있습니다.\n');
    fprintf('   - 수치적 안정화를 위해 행렬에 작은 정규화 항 추가 (A + εI)\n');
    fprintf('   - 단위계 조정을 통한 조건수 개선\n');
end

fprintf('2. 하이브리드 접근법 고려:\n');
fprintf('   - 시간이 중요한 경우 수치적 선형화 사용\n');
fprintf('   - 오프라인 분석 및 제어기 설계 시 심볼릭 선형화 사용\n');
fprintf('   - 계산 전에 조건수 체크 후 방법 선택\n');

fprintf('3. 선형화 정확도 향상을 위한 고려사항:\n');
fprintf('   - 더 작은 시간 스텝 사용\n');
fprintf('   - 고차 테일러 전개 고려\n');
fprintf('   - 비선형성이 강한 영역에서 지역 선형화 모델 더 자주 업데이트\n');

%% 수치적 선형화 함수 (필요시 구현)
function [A, B] = linearize_HANul_numerical(X, U, params, dt)
    % linearize_HANul_numerical: HANul 로켓용 수치적 선형화 함수
    %
    % 입력:
    %   X: 현재 상태 벡터 [pos; V_B; att_quat; omega] (13x1)
    %   U: 제어 입력 [카나드1-4, RCS1-4] (8x1)
    %   params: 로켓 파라미터 구조체
    %   dt: 시간 스텝 [s]
    %
    % 출력:
    %   A: 상태 행렬 (쿼터니언+각속도) [7x7]
    %   B: 입력 행렬 (쿼터니언+각속도) [7x8]
    
    % 쿼터니언 및 각속도 관련 상태 인덱스
    quat_indices = 7:10;
    omega_indices = 11:13;
    state_indices = [quat_indices, omega_indices];
    
    % 수치적 자코비안 계산을 위한 섭동 크기
    eps = 1e-6;
    
    % 기준점에서 시스템 응답 계산
    [X_next_base, ~, ~, ~, ~, ~, ~] = srk4(@vehicle_dynamics_HANul, X, U, zeros(6,1), dt, params, 0, dt);
    X_next_base = X_next_base(state_indices);
    
    % A 행렬 계산 (상태에 대한 자코비안)
    A = zeros(7, 7);
    for i = 1:7
        % 상태 섭동 벡터
        dX = zeros(size(X));
        
        % 쿼터니언 정규화를 위한 특별 처리
        if i <= 4
            % 쿼터니언 요소 섭동
            dX(quat_indices(i)) = eps;
            
            % 섭동된 쿼터니언 정규화
            q_perturbed = X(quat_indices) + dX(quat_indices);
            q_perturbed = q_perturbed / norm(q_perturbed);
            
            % 정규화된 쿼터니언으로 X_perturbed 생성
            X_perturbed = X;
            X_perturbed(quat_indices) = q_perturbed;
        else
            % 각속도 요소 섭동
            dX(omega_indices(i-4)) = eps;
            X_perturbed = X + dX;
        end
        
        % 섭동된 상태에서 시스템 응답 계산
        [X_next_perturbed, ~, ~, ~, ~, ~, ~] = srk4(@vehicle_dynamics_HANul, X_perturbed, U, zeros(6,1), dt, params, 0, dt);
        X_next_perturbed = X_next_perturbed(state_indices);
        
        % 수치 미분을 통한 자코비안 계산
        delta_response = X_next_perturbed - X_next_base;
        
        % 쿼터니언 정규화 영향 고려
        if i <= 4
            A(:, i) = delta_response / (q_perturbed(i) - X(quat_indices(i)));
        else
            A(:, i) = delta_response / eps;
        end
    end
    
    % B 행렬 계산 (입력에 대한 자코비안)
    B = zeros(7, 8);
    for i = 1:8
        % 입력 섭동 벡터
        dU = zeros(size(U));
        dU(i) = eps;
        
        % 섭동된 입력으로 시스템 응답 계산
        [X_next_perturbed, ~, ~, ~, ~, ~, ~] = srk4(@vehicle_dynamics_HANul, X, U + dU, zeros(6,1), dt, params, 0, dt);
        X_next_perturbed = X_next_perturbed(state_indices);
        
        % 수치 미분을 통한 자코비안 계산
        delta_response = X_next_perturbed - X_next_base;
        B(:, i) = delta_response / eps;
    end
    
    % 수치적 안정성을 위한 작은 값 제거
    A(abs(A) < 1e-10) = 0;
    B(abs(B) < 1e-10) = 0;
end