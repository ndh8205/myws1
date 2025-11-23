function [U, computation_time, predicted_trajectory] = Controller_NMPC(X, X_target, dt, params)
    % NMPC 매개변수
    Np = 5; % 예측 지평선
    Nc = 5; % 제어 지평선

    % 이전 최적 제어 입력을 사용하기 위한 persistent 변수
    persistent U_prev
    if isempty(U_prev)
        U_prev = zeros(8 * Nc, 1);
    end

    % 초기 제어 입력 추정 (이전 최적 제어 입력을 사용)
    U0 = U_prev;

    % fmincon 옵션 설정 (병렬 처리 추가)
    % options = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'off', ...
    %                        'UseParallel', true, 'MaxIterations', 200);

    options = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'off', ...
                           'UseParallel', false , 'MaxIterations', 200);


    % 제약 조건 설정
    lb = zeros(8 * Nc, 1); % 하한
    ub = ones(8 * Nc, 1); % 상한

    % 계산 시간 측정 시작
    tic;

    % fmincon을 사용한 최적화 (병렬 처리 사용)
    [U_opt, ~, exitflag, output] = fmincon(@(U) cost_function(U, X, X_target, Np, Nc, dt, params), ...
        U0, [], [], [], [], lb, ub, ...
        @(U) nonlinear_constraints(U, X, Np, Nc, dt, params), options);

    % 계산 시간 측정 종료
    computation_time = toc;
    disp(['Computation time: ', num2str(computation_time), ' seconds']);
    disp(['Number of iterations: ', num2str(output.iterations)]);

    % 최적 제어 입력 추출
    U = reshape(U_opt(1:8), [8, 1]);

    % 다음 단계를 위해 최적 제어 입력 저장
    U_prev = U_opt;

    % 예측 궤적 계산
    predicted_trajectory = predict_trajectory(U_opt, X, Np, Nc, dt, params);
end


function J = cost_function(U, X0, X_target, Np, Nc, dt, params)
% 비용 함수 계산
 Q = diag([10, 10, 10, 10, 10, 10]); % 상태 가중치
 R = 0.01 * eye(8); % 제어 가중치
 P = 1 * Q; % 터미널 가중치
 J = 0;
 X = X0;
 U_reshaped = reshape(U, [8, Nc]);

    for k = 1:Np
    
        if k <= Nc
         u_k = U_reshaped(:, k);
        else
         u_k = U_reshaped(:, end);
        end
     X = rk4(@vehicle_dynamics_Airbearing, X, u_k, dt, params);
    
     state_error = X - X_target;
    
     J = J + state_error' * Q * state_error + u_k' * R * u_k;
    
    end

 final_state_error = X - X_target;
 J = J + final_state_error' * P * final_state_error;

end

function [c, ceq] = nonlinear_constraints(U, X0, Np, Nc, dt, params)

% 비선형 제약 조건
 X_pred = predict_trajectory(U, X0, Np, Nc, dt, params);

% 불등식 제약 조건 (c <= 0)
 c = [];

    for k = 1:size(X_pred, 2)
     c = [c;
     -X_pred(1,k); % x >= 0
     X_pred(1,k) - 1000; % x <= 1000
     -X_pred(2,k); % y >= 0
     X_pred(2,k) - 1000]; % y <= 1000
    end

% 등식 제약 조건 (ceq = 0)
 ceq = [];

end

function X_pred = predict_trajectory(U, X0, Np, Nc, dt, params)

 X_pred = zeros(6, Np+1);
 X_pred(:,1) = X0;
 U_reshaped = reshape(U, [8, Nc]);

    for k = 1:Np
        if k <= Nc
         u_k = U_reshaped(:, k);
        else
         u_k = U_reshaped(:, end);
        end
         X_pred(:,k+1) = rk4(@vehicle_dynamics_Airbearing, X_pred(:,k), u_k, dt, params);
    end
end