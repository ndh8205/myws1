function [command_Vector_chaser, command_Vector_target, msp] = generate_commands_rendezvous3(X_chaser, X_target)
    persistent path_points current_path_index prev_target_state;
    
    % 첫 실행시 초기화
    if isempty(path_points) || isempty(current_path_index) || isempty(prev_target_state)
        path_points = [];
        current_path_index = 1;
        prev_target_state = [];
    end
    
    % FOV 파라미터
    fov_angle = deg2rad(55);
    max_range = 2000;
    
    % 타겟의 현재 상태
    target_pos = X_target(1:2);
    target_heading = X_target(5);
    current_target_state = [target_pos; target_heading];
    
    % 타겟이 이동했거나 처음 실행시 새로운 경로 계획
    if isempty(path_points) || isempty(prev_target_state) || ...
       any(abs(current_target_state - prev_target_state) > 1e-6)
        % 최적 경로 계산
        path_points = plan_optimal_path(X_chaser(1:2), target_pos, target_heading, ...
                                      fov_angle, max_range);
        current_path_index = 1;
        prev_target_state = current_target_state;
    end
    
    % 현재 목표점 가져오기
    current_target = path_points(:, current_path_index);
    
    % 현재 목표점까지의 거리
    rel_pos = current_target - X_chaser(1:2);
    dist = norm(rel_pos);
    
    % 목표점에 도달했는지 확인 (waypoint 갱신)
    if dist < 50  % 50mm 이내면 다음 waypoint로
        current_path_index = min(current_path_index + 1, size(path_points, 2));
        current_target = path_points(:, current_path_index);
        rel_pos = current_target - X_chaser(1:2);
        dist = norm(rel_pos);
    end
    
    % 속도 프로파일 생성 (부드러운 가감속)
    approach_speed = calculate_smooth_velocity(dist, current_path_index, ...
                                            size(path_points, 2));
    
    % 접근 방향 계산
    approach_dir = rel_pos / max(dist, 1e-6);  % 0으로 나누기 방지
    
    % 위치와 속도 명령
    pos_chaser = [current_target; approach_dir * approach_speed];
    
    % 다음 waypoint를 고려한 선행 자세 계산
    desired_angle = calculate_predictive_attitude(X_chaser, current_path_index, ...
                                               path_points, target_pos);
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

function path_points = plan_optimal_path(start_pos, target_pos, target_heading, fov_angle, max_range)
    % FOV 경계점 찾기
    [fov_point, ~] = find_nearest_fov_point(start_pos, target_pos, target_heading, ...
                                          fov_angle, max_range);
    
    % 경로 포인트 생성 (베지어 곡선 사용)
    num_points = 10;  % 경로 포인트 개수
    
    % 제어점 설정
    control_points = [...
        start_pos, ...                                    % 시작점
        start_pos + 0.5 * (fov_point - start_pos), ...   % 중간 제어점 1
        fov_point + [-200; 0], ...                       % 중간 제어점 2
        fov_point ...                                     % 끝점
    ];
    
    % 베지어 곡선 생성
    path_points = zeros(2, num_points);
    for i = 1:num_points
        t = (i-1)/(num_points-1);
        path_points(:,i) = cubic_bezier(control_points, t);
    end
end

function point = cubic_bezier(control_points, t)
    % 3차 베지어 곡선 계산
    p0 = control_points(:,1);
    p1 = control_points(:,2);
    p2 = control_points(:,3);
    p3 = control_points(:,4);
    
    point = p0 * (1-t)^3 + ...
            3 * p1 * t * (1-t)^2 + ...
            3 * p2 * t^2 * (1-t) + ...
            p3 * t^3;
end

function v = calculate_smooth_velocity(dist, current_index, total_points)
    % 부드러운 속도 프로파일 생성
    base_speed = 2.0;  % 기본 속도
    
    % 거리 기반 속도 조절
    if dist > 500
        dist_factor = 1.0;
    else
        dist_factor = 0.5 + 0.5 * (dist/500);
    end
    
    % 경로 진행도 기반 속도 조절
    progress = current_index / total_points;
    if progress < 0.2  % 가속 구간
        progress_factor = 0.5 + 2.5 * progress;
    elseif progress > 0.8  % 감속 구간
        progress_factor = 2.5 * (1 - progress);
    else  % 정속 구간
        progress_factor = 1.0;
    end
    
    v = base_speed * dist_factor * progress_factor;
end

function angle = calculate_predictive_attitude(X_chaser, current_index, path_points, target_pos)
    % 선행 자세 계산 (다음 waypoint를 고려)
    next_index = min(current_index + 1, size(path_points, 2));
    next_point = path_points(:, next_index);
    
    % 현재 위치에서 다음 waypoint까지의 방향과
    % 최종 목표물 방향을 가중 평균하여 부드러운 회전
    path_direction = atan2(next_point(2) - X_chaser(2), next_point(1) - X_chaser(1));
    target_direction = atan2(target_pos(2) - X_chaser(2), target_pos(1) - X_chaser(1));
    
    % 경로 진행도에 따른 가중치
    progress_weight = current_index / size(path_points, 2);
    angle = (1 - progress_weight) * path_direction + progress_weight * target_direction;
end