delete(gcp('nocreate'))
clear all
close all
clc

parpool('local'); % 병렬 연산 풀 생성

% 프로세스 노이즈 시그마 값들의 범위 설정
sigma_u_values = linspace(0.1, 50.0, 10); % u 방향 속도 노이즈
sigma_v_values = linspace(0.1, 50.0, 10); % v 방향 속도 노이즈
sigma_r_values = linspace(deg2rad(0.1), deg2rad(1.0), 5); % 회전 속도 노이즈

params = Params_init();

% 시그마 값 조합 생성
[sigma_u_grid, sigma_v_grid, sigma_r_grid] = ndgrid(sigma_u_values, sigma_v_values, sigma_r_values);
sigma_u_list = sigma_u_grid(:);
sigma_v_list = sigma_v_grid(:);
sigma_r_list = sigma_r_grid(:);

num_combinations = numel(sigma_u_list);
errors = zeros(num_combinations, 1);

parfor idx = 1:num_combinations
    sigma_u = sigma_u_list(idx);
    sigma_v = sigma_v_list(idx);
    sigma_r = sigma_r_list(idx);

    % 시뮬레이션 실행
    [error, ~, ~] = run_simulation(sigma_u, sigma_v, sigma_r, params);

    errors(idx) = error;
    fprintf('Combination %d/%d: Error = %.4f\n', idx, num_combinations, error);
end

% 최적의 시그마 값 찾기
[best_error, best_idx] = min(errors);
best_sigma_u = sigma_u_list(best_idx);
best_sigma_v = sigma_v_list(best_idx);
best_sigma_r = sigma_r_list(best_idx);

fprintf('Best sigma_u: %.2f\n', best_sigma_u);
fprintf('Best sigma_v: %.2f\n', best_sigma_v);
fprintf('Best sigma_r: %.2f degrees\n', rad2deg(best_sigma_r));
fprintf('Best RMSE: %.4f\n', best_error);
