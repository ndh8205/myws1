function [command_Vector_chaser, command_Vector_target, msp] = generate_commands_rendezvous2(X_chaser, X_target)
    persistent fixed_target_point prev_target_state;
    
    % 첫 실행시 초기화
    if isempty(fixed_target_point) || isempty(prev_target_state)
        fixed_target_point = [];
        prev_target_state = [];
    end
    
    % FOV 파라미터
    fov_angle = deg2rad(55); % 시야각
    max_range = 2000; % 최대 감지 거리
    
    % 타겟의 현재 상태
    target_pos = X_target(1:2);
    target_heading = X_target(5);
    current_target_state = [target_pos; target_heading];
    
    % 타겟이 처음이거나 이동했을 때만 새로운 목표점 계산
    if isempty(fixed_target_point) || isempty(prev_target_state) || ...
       any(abs(current_target_state - prev_target_state) > 1e-6)
        [fixed_target_point, ~] = find_nearest_fov_point(X_chaser(1:2), target_pos, ...
                                                       target_heading, fov_angle, max_range);
        prev_target_state = current_target_state;
    end
    
    % 현재 위치에서 고정된 목표점까지의 상대 벡터
    rel_pos = fixed_target_point - X_chaser(1:2);
    dist = norm(rel_pos);
    
    % 접근 속도 설정
    if dist > 500
        approach_speed = 2.0;
    else
        approach_speed = 1.0;
    end
    
    % 접근 방향 계산
    approach_dir = rel_pos / dist;
    
    % 위치와 속도 명령
    pos_chaser = [fixed_target_point; approach_dir * approach_speed];
    
    % 타겟을 바라보는 자세 명령
    desired_angle = atan2(target_pos(2) - X_chaser(2), target_pos(1) - X_chaser(1));
    att_chaser = [desired_angle; 0];
    
    % 최종 명령 벡터
    command_Vector_chaser = [pos_chaser; att_chaser];
    command_Vector_target = zeros(6,1);
    msp = 1;
end

function [nearest_point, is_range_limited] = find_nearest_fov_point(chaser_pos, target_pos, target_heading, fov_angle, max_range)
    % 체이서에서 타겟까지의 상대 벡터
    rel_pos = chaser_pos - target_pos;
    dist = norm(rel_pos);
    
    % 타겟까지의 절대 각도
    abs_angle = atan2(rel_pos(2), rel_pos(1));
    
    % 타겟 방향을 기준으로 한 상대 각도
    rel_angle = wrapToPi(abs_angle - target_heading);
    
    % FOV 경계 각도
    fov_left = target_heading - fov_angle/2;
    fov_right = target_heading + fov_angle/2;
    
    % 거리 제한 확인
    is_range_limited = dist > max_range;
    
    if is_range_limited
        % 최대 거리에서 가장 가까운 점 찾기
        scaled_pos = target_pos + max_range * [cos(abs_angle); sin(abs_angle)];
        nearest_point = scaled_pos;
    else
        % 각도에 따른 FOV 경계점 찾기
        if rel_angle < -fov_angle/2
            nearest_angle = fov_left;
        elseif rel_angle > fov_angle/2
            nearest_angle = fov_right;
        else
            % 이미 FOV 안에 있는 경우 (이런 경우는 없어야 함)
            nearest_angle = abs_angle;
        end
        
        % FOV 경계상의 가장 가까운 점 계산
        nearest_point = target_pos + dist * [cos(nearest_angle); sin(nearest_angle)];
    end
end