function [command_Vector, msp] = generate_commands(real_time, current_state)
    % 도킹 대상의 위치와 자세
    target_pos = [ 2580.22; 1600.95 ];      % [mm]
    target_attitude = deg2rad(90);   % [rad] 도킹 대상의 자세 (y축은 -x방향)
    approach_angle = deg2rad(0);    % [rad] 원하는 입사각
    
    % Pre-docking point 계산 
    R_pre = 1200;  % [mm]
    
    % -x 방향에서 반시계방향으로 15도 틀어진 위치
    predocking_x = target_pos(1) + R_pre * cos(pi + approach_angle);
    predocking_y = target_pos(2) + R_pre * sin(pi + approach_angle);
    
    desired_attitude = deg2rad(-90) + approach_angle;
    
    persistent alignment_achieved
    if isempty(alignment_achieved)
        alignment_achieved = false;
    end
    
    if ~alignment_achieved
        pos_error = sqrt((current_state(1) - predocking_x)^2 + (current_state(2) - predocking_y)^2);
        attitude_error = abs(wrapToPi(current_state(5) - desired_attitude));
        
        pos_target = [predocking_x; predocking_y; 0; 0];
        att_target = [desired_attitude; deg2rad(0)];
        msp = 1;
        
        if pos_error < 50 && attitude_error < deg2rad(5)
            alignment_achieved = true;
        end
    else
        pos_target = [target_pos(1); target_pos(2); 0; 0];
        att_target = [desired_attitude; deg2rad(0)];
        msp = 3;
    end
    
    command_Vector = [pos_target; att_target];
end