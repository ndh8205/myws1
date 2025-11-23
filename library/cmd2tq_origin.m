function torque = cmd2tq_origin( command, idle, params, dt )

    % Convert reaction wheel command (0-98) to torque
    % 0 is max negative torque
    % 98 is max positive torque
    
    omega_limit = params.Reaction_wheel.omega_limit;
    I_wheel = params.Reaction_wheel.I_wheel;
    MAA = omega_limit / dt;
    max_torque = I_wheel/2 * MAA;
        
    if command == idle

        torque = 0;

    elseif command < idle

        torque = -max_torque * (idle - command) / idle;

    else

        torque = max_torque * (command - idle) / idle;

    end

end