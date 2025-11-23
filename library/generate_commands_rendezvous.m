% 마스터 포인트 접근 명령 생성 함수
function [command_Vector_chaser, command_Vector_target, msp] = generate_commands_rendezvous(X_chaser, master_point)
    % 마스터 포인트 기준 접근 설정
    target_distance = 1000;  % 마스터 포인트로부터 목표 거리
    master_direction = -pi/2;  % 마스터 포인트 방향 (아래쪽)
    
    % 목표점 계산 (마스터 포인트 아래 1000mm)
    target_pos = master_point(1:2) + target_distance * [cos(master_direction); sin(master_direction)];
    
    % 현재 위치에서 목표점까지의 상대 벡터
    rel_pos = target_pos - X_chaser(1:2);
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
    pos_chaser = [target_pos; approach_dir * approach_speed];
    
    % 마스터 포인트를 바라보는 자세 명령
    desired_angle = atan2(master_point(2) - X_chaser(2), master_point(1) - X_chaser(1));
    att_chaser = [desired_angle; 0];
    
    % 최종 명령 벡터
    command_Vector_chaser = [pos_chaser; att_chaser];
    command_Vector_target = zeros(6,1);  % 표적 위성 명령은 빈 벡터
    msp = 1;  % 마스터 포인트 접근 모드
end