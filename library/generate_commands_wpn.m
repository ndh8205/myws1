function [command_Vector, msp] = generate_commands_wpn(real_time)
    %--- (1) 원의 중심, 반지름 및 각속도 설정 ---%
    centerX = 1500;
    centerY = 1500;
    R = 1100;         % 원의 반지름 (1m)
    omega = 0.025;    % 각속도 (rad/s)

    if real_time <= 30
        % 30초간 정렬: 9시 위치에서 고정 (즉, center에서 왼쪽)
        posX = centerX - R;  % 9시 위치: centerX - R
        posY = centerY  ;

        % 원의 중심을 향하는 각도 계산
        deltaX = centerX - posX;
        deltaY = centerY - posY;
        alpha = atan2(deltaY, deltaX);
        % 로봇의 Y축이 중심을 향하도록 heading 계산 후 래핑 적용
        heading = wrapToPi(alpha - pi/2);

        pos_target = [posX; posY; 0; 0];    % (x, y, vx, vy)
        att_target = [heading; 0];           % (yaw, yaw_rate)

        msp = 1;
    else
        % 30초 이후: 원 궤적을 따라 이동 시작
        t_move = real_time - 30;   % 이동 시작 시간 기준

        % 원 파라미터화: 시작각 theta = pi (즉, 9시 위치)에서 출발
        theta = pi + omega * t_move;
        posX = centerX + R * cos(theta);
        posY = centerY + R * sin(theta);

        % 원의 중심을 향하는 각도 계산
        deltaX = centerX - posX;
        deltaY = centerY - posY;
        alpha = atan2(deltaY, deltaX);
        heading = wrapToPi(alpha - pi/2);

        pos_target = [posX; posY; 0; 0];
        att_target = [heading; 0];

        msp = 1;
    end

    % 최종 명령 벡터: [x; y; vx; vy; yaw; yaw_rate]
    command_Vector = [pos_target; att_target];
end
