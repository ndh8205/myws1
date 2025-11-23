function [X, weights, x_est] = particle_filter(z, X, weights, Ut, dt, X_target, params)
    % Particle Filter with the same measurement model

    num_particles = size(X, 2);
    X_pred = zeros(size(X));

    % Qk def
    qs_u = 15;
    qs_v = 15;
    qs_r = deg2rad(35);
    qs_x = 0;
    qs_y = 0;
    qs_psi = deg2rad(0);

    Qs = diag( [ qs_x.^2, qs_y.^2, qs_u.^2, qs_v.^2, qs_psi.^2, qs_r.^2 ] );
    
    q_x = Qs(1,1);
    q_y = Qs(2,2);
    q_u = Qs(3,3);
    q_v = Qs(4,4);
    q_psi = Qs(5,5);
    q_r = Qs(6,6);

    % Define Qk matrix
    Qk = zeros(6,6);

    % Define all elements
    Qk(1,1) = dt^3*((q_v*sin(X(5))^2)/3 - (q_u*(sin(X(5))^2 - 1))/3);
    Qk(1,2) = (dt^3*sin(2*X(5))*(q_u - q_v))/6;
    Qk(1,3) = (X(6)*q_v*sin(X(5))*dt^3)/3 + (q_u*cos(X(5))*dt^2)/2;
    Qk(1,4) = (X(6)*q_u*cos(X(5))*dt^3)/3 - (q_v*sin(X(5))*dt^2)/2;
    Qk(1,5) = 0;
    Qk(1,6) = 0;

    Qk(2,1) = Qk(1,2);
    Qk(2,2) = dt^3*((q_u*sin(X(5))^2)/3 - (q_v*(sin(X(5))^2 - 1))/3);
    Qk(2,3) = - (X(6)*q_v*cos(X(5))*dt^3)/3 + (q_u*sin(X(5))*dt^2)/2;
    Qk(2,4) = (X(6)*q_u*sin(X(5))*dt^3)/3 + (q_v*cos(X(5))*dt^2)/2;
    Qk(2,5) = 0;
    Qk(2,6) = 0;

    Qk(3,1) = Qk(1,3);
    Qk(3,2) = Qk(2,3);
    Qk(3,3) = ((q_v*X(6)^2)/3 + (q_r*X(4)^2)/3)*dt^3 + q_u*dt;
    Qk(3,4) = - (q_r*X(3)*X(4)*dt^3)/3 + ((X(6)*q_u)/2 - (X(6)*q_v)/2)*dt^2;
    Qk(3,5) = -(dt^3*q_r*X(4))/3;
    Qk(3,6) = -(dt^2*q_r*X(4))/2;

    Qk(4,1) = Qk(1,4);
    Qk(4,2) = Qk(2,4);
    Qk(4,3) = Qk(3,4);
    Qk(4,4) = ((q_u*X(6)^2)/3 + (q_r*X(3)^2)/3)*dt^3 + q_v*dt;
    Qk(4,5) = (dt^3*q_r*X(3))/3;
    Qk(4,6) = (dt^2*q_r*X(3))/2;

    Qk(5,1) = Qk(1,5);
    Qk(5,2) = Qk(2,5);
    Qk(5,3) = Qk(3,5);
    Qk(5,4) = Qk(4,5);
    Qk(5,5) = (dt^3*q_r)/3;
    Qk(5,6) = (dt^2*q_r)/2;

    Qk(6,1) = Qk(1,6);
    Qk(6,2) = Qk(2,6);
    Qk(6,3) = Qk(3,6);
    Qk(6,4) = Qk(4,6);
    Qk(6,5) = Qk(5,6);
    Qk(6,6) = dt*q_r;

    % measurement noise matrix
    Rq_rho = 0.05;
    Rq_theta = deg2rad(0.01);
    Rq_rm = deg2rad(5);
    Rs = diag( [ Rq_rho.^2, Rq_theta.^2, Rq_rm.^2 ] );
    R = Rs;

    % 예측 단계
    for i = 1:num_particles
        X = X(:, i);

        % 프로세스 노이즈 샘플링
        w = mvnrnd(zeros(size(X)), Qk)';

        % 상태 예측 (비선형 동역학 모델 사용)
        xdot = vehicle_dynamics_Airbearing_stochastic(X, Ut, w(3:5), dt, params);
        X_pred(:, i) = X + xdot * dt + [w(1:2); zeros(4,1)];
    end

    % 업데이트 단계
    for i = 1:num_particles
        X = X_pred(:, i);

        % 측정 예측
        h = measurement_model(X, X_target);

        % 측정 노이즈에 따른 likelihood 계산
        v = z - h;
        weights(i) = weights(i) * mvnpdf(v', zeros(length(z),1)', R);
    end

    % 가중치 정규화
    weights = weights / sum(weights);

    % 재샘플링 필요 여부 확인 (효과적 파티클 수 계산)
    Neff = 1 / sum(weights.^2);
    N_threshold = num_particles / 2;

    if Neff < N_threshold
        % 재샘플링 수행
        indices = systematic_resample(weights);
        X = X_pred(:, indices);
        weights = ones(1, num_particles) / num_particles;
    else
        % 재샘플링 없이 파티클 업데이트
        X = X_pred;
    end

    % 상태 추정값 계산 (가중치 평균)
    x_est = X * weights';

end

% 측정 모델 함수
function h = measurement_model(X, X_target)
    % 상태 변수 추출
    x = X(1);
    y = X(2);
    psi = X(5);
    r = X(6);

    % 상대 위치 계산
    dx = X_target(1) - x;
    dy = X_target(2) - y;

    % 몸체 좌표계로 변환
    RI2B = [ cos( psi ), sin( psi ); -sin( psi ), cos( psi ) ];
    dxy_body = RI2B * [ dx; dy ];

    % rho와 theta 계산
    rho = sqrt( dxy_body(1)^2 + dxy_body(2)^2 );
    theta = atan2( dy, dx ) - psi;
    theta = wrapToPi(theta);

    % h(x) 구성
    h = [ rho; theta; r ];
end

% 시스템 재샘플링 함수
function indices = systematic_resample(weights)
    N = length(weights);
    positions = (0:N-1) / N + rand / N;
    indices = zeros(1,N);
    cumulative_sum = cumsum(weights);
    i = 1;
    j = 1;
    while i <= N
        if positions(i) < cumulative_sum(j)
            indices(i) = j;
            i = i + 1;
        else
            j = j + 1;
        end
    end
end
