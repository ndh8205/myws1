function z_measured = measure_sensor( Xk, X_target, Rk )
    % 측정 가능한 변수들만 포함하는 z_measured 벡터 생성
    z_measured = zeros(3, 1);

    % 위성과 타겟 사이의 거리 (ρ) 계산 및 측정
    dx = X_target(1) - Xk(1);
    dy = X_target(2) - Xk(2);
    true_distance = sqrt(dx^2 + dy^2);
    z_measured(1) = true_distance + randn * sqrt(Rk(1,1));

    % 위성 기준 타겟의 상대 방위각 (θ) 계산 및 측정
    true_relative_angle = atan2(dy, dx) - Xk(5);  % Xk(5)는 위성의 자세각
    z_measured(2) = wrapToPi(true_relative_angle) + randn * sqrt(Rk(2,2));

    % 위성의 각속도 (r) 측정
    z_measured(3) = Xk(6) + randn * sqrt(Rk(3,3));
end