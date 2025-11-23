function [command_Vector_chaser, command_Vector_target, msp] = generate_commands_dock(real_time, X_chaser, X_target)
    % 타겟의 위치
    target_pos = X_target(1:2);    % [2800; 1500]

    % 중심선(Y=1500) 까지의 거리
    center_line_Y = 1500;  % 고정된 중심선 Y좌표
    Y_error = X_chaser(2) - center_line_Y;

    % X축 방향 거리
    X_dist = target_pos(1) - X_chaser(1);

    % 중심선 도달 판단 (허용 오차)
    on_center_line = abs(Y_error) < 30;  % 30mm 이내를 중심선으로 판단

    % 위치와 속도 명령 생성
    if ~on_center_line
        % 중심선에 없을 때: Y축 방향 이동 우선
        target_Y = center_line_Y;  % 목표 Y 위치를 중심선으로
        target_Y_dist = target_pos(2) - X_chaser(2);
        Vy = sign(Y_error)*2;  % Y축 방향 속도
        target_X = X_chaser(1) + sqrt(target_Y_dist);    % 타겟의 X 위치로 접근

        Vx = -sign(X_dist)*2;

        % target_X = X_chaser(1);    % 타겟의 X 위치로 접근
        % Vx = 0;
        msp = 1;
        % fprintf('Moving to center line. Y error: %.2f\n', Y_error);
    else
        % 중심선에 있을 때: X축 방향 접근
        target_Y = center_line_Y;  % 중심선 유지
        target_X = X_target(1) - 100;    % 타겟의 X 위치로 접근

        % X 방향 접근 속도 설정
        if abs(X_dist) > 1000
            Vx = sign(X_dist) * 2.0;
        elseif abs(X_dist) > 500
            Vx = sign(X_dist) * 1.0;
        else
            Vx = sign(X_dist) * 0.5;
        end
        Vy = -sign(Y_error) * 0.2;  % 작은 Y방향 보정 유지
        msp = 2;
        % fprintf('On center line. Moving to target. X distance: %.2f\n', X_dist);
    end

    % 위치 명령 - 목표 위치로 설정
    pos_chaser = [target_X; target_Y; Vx; Vy];

    % 자세 명령 - 항상 타겟과 키스 자세 유지
    att_chaser = [wrapToPi(X_target(5) + pi); 0];

    % 최종 명령 벡터 생성
    command_Vector_chaser = [pos_chaser; att_chaser];
    command_Vector_target = [target_pos; 0; 0; X_target(5); 0];
end
