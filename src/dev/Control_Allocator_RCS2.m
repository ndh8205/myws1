function [final_cmd, thruster_cmd] = Control_Allocator_RCS2(torque_cmd, params)

    T = params.vehicle.RCS_T;   % [N]
    d = params.vehicle.r_ref;   % [m]
    l = params.vehicle.Lrcs;    % [m]

    R_RCS = T * d;  % Roll axis torque per thruster
    L_RCS = T * l;  % Pitch and Yaw axis torque per thruster

    % T1       T2        T3        T4        T5        T6        T7        T8
    B = [  R_RCS, -R_RCS,  R_RCS, -R_RCS,  R_RCS, -R_RCS,  R_RCS, -R_RCS;  % Roll
           -L_RCS,      0,      0, -L_RCS,  L_RCS,      0,      0,  L_RCS;  % Pitch
                0, -L_RCS,  L_RCS,      0,      0, -L_RCS,  L_RCS,      0]; % Yaw

    % torque binary combination set
    combinations = dec2bin(0:255) - '0';
    num_combinations = size(combinations, 1);

    % cmd torque - error
    min_error = Inf;
    final_cmd = zeros(8, 1);

    for i = 1:num_combinations
        cmd = combinations(i, :)';
        produced_torque = B * cmd;
        error = norm(torque_cmd - produced_torque);

        if error < min_error
            min_error = error;
            final_cmd = cmd;
        end
    end

    % 출력 추력기 명령
    thruster_cmd = final_cmd

end
