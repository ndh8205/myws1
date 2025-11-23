function in_fov = check_fov(X_chaser, X_target)
    % 입력값 검증
    if any(isnan(X_chaser)) || any(~isreal(X_chaser)) || ...
       any(isnan(X_target)) || any(~isreal(X_target))
        in_fov = false;
        return;
    end
    
    % 실수부만 사용
    X_chaser = real(X_chaser);
    X_target = real(X_target);
    
    % FOV 파라미터
    fov_angle = deg2rad(55); % 시야각 (55도)
    max_range = 2000; % 최대 감지 거리 (mm)
    
    % 타겟 기준 체이서까지의 상대 위치
    rel_pos = X_chaser(1:2) - X_target(1:2);
    
    % 거리가 0인 경우 처리
    if norm(rel_pos) < 1e-6
        in_fov = true;
        return;
    end
    
    try
        distance = norm(rel_pos);
        % y축 기준으로 target_heading 설정 (0도가 y축 방향)
        target_heading = X_target(5); % 이미 y축 기준이면 그대로 사용
        
        % 체이서까지의 절대 각도 (y축 기준)
        abs_angle = -atan2(rel_pos(1), rel_pos(2)); % x, y 순서 변경
        
        % 타겟 방향을 중심으로 한 상대 각도
        rel_angle = wrapToPi(abs_angle - target_heading);
        
        % FOV 판단
        in_fov = (abs(rel_angle) <= fov_angle/2) && (distance <= max_range);
    catch
        in_fov = false;
    end
end