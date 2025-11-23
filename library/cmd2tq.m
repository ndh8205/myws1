function torque = cmd2tq(scaled_command, idle, params, dt)

    % 리액션 휠 파라미터 추출
    I_wheel = params.Reaction_wheel.I_wheel; % [kg*mm^2]
    omega_limit = params.Reaction_wheel.omega_limit; % [rad/s]

    % 지속 변수 설정: 리액션 휠의 현재 각속도
    persistent omega_current

    if isempty(omega_current)
        omega_current = params.Reaction_wheel.omega_wheel_init; % 초기 각속도 설정
    end

    % 스케일링된 명령 값을 -1 ~ 1 사이로 정규화
    normalized_command = (scaled_command - idle) / idle;
    % 최대 값이 98이므로, 스케일링을 조정하여 49를 기준으로 -1 ~ 1 범위로 만듭니다.

    % 목표 각속도 계산
    omega_target = normalized_command * omega_limit;

    % 각속도 변화율 계산
    omega_dot = (omega_target - omega_current) / dt;

    % 토크 계산
    torque = I_wheel * omega_dot; % [N*m] 또는 단위에 따라 [N*mm]

    % 각속도 업데이트
    omega_current = omega_current + omega_dot * dt;

    % 각속도 제한 적용
    if abs(omega_current) > omega_limit
        omega_current = sign(omega_current) * omega_limit;
    end
    
end