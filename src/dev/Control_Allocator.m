function U = Control_Allocator( Tau, params, flight_phase )
    % Calculate average thrust
    F_T = mean(params.motor.thrust_func_stage2);
    
    % Extract required parameters
    l_nozzle = abs(params.rocket.x_Nozzle);
    I_w = params.reaction_wheel.I_wheel;
    max_f_d = params.fin.max_angle;
    max_ga = params.rocket.gimbal_limmit;
    max_o = params.reaction_wheel.omega_limit;

    FL = F_T * abs( l_nozzle );
    
    % Define control matrix A based on flight phase

    switch flight_phase

        case 'First_burn'
            % TVC and RW only
            A = [
                0,   0,   0,   0,   0,   0,  0;
                0,   0,   0,   0,  FL,   0,  0;
                0,   0,   0,   0,   0,  FL, I_w
                ];
            disp("First_burn")

        case 'No_burn'
            % Fin and RW only
            A = [
                -1/4, -1/4,  1/4,  1/4,   0,  0,  0;
                -1/4, -1/4, -1/4, -1/4,   0,  0,  0;
                 1/4, -1/4, -1/4,  1/4,   0,  0, I_w
                ];
            disp("No_burn")

        case 'landing'
            % Full control: TVC, Fin, and RW
            A = [
                -1/4, -1/4,  1/4,  1/4,  0,  0,  0;
                -1/4, -1/4, -1/4, -1/4, FL,  0,  0;
                 1/4, -1/4, -1/4,  1/4,  0, FL, I_w
                ];
            disp("landing")

        otherwise
            error('Unknown flight phase');
    end
    
    % Solve for control inputs
    U = pinv(A) * Tau;
    
    % Extract control inputs
    fin_deflections = U(1:4);
    gamma1 = U(5);
    gamma2 = U(6);
    delta_omega = U(7);
    
    % Apply limits using LIMIT2 function
    for i = 1:4
        fin_deflections(i) = LIMIT2(fin_deflections(i), -max_f_d, max_f_d);
    end

    gamma1 = LIMIT2(gamma1, -max_ga, max_ga);
    gamma2 = LIMIT2(gamma2, -max_ga, max_ga);
    
    disp("gamma1 : ")
    disp(rad2deg(gamma1))

    delta_omega = LIMIT2(delta_omega, -max_o, max_o);
    
    % Reconstruct limited control vector

    U = [fin_deflections; gamma1; gamma2; delta_omega];
    
    % Set unused controls to zero based on flight phase
    if strcmp(flight_phase, 'ascent')

        U(1:4) = 0;  % Set fin deflections to zero

    elseif strcmp(flight_phase, 'descent')

        U(5:6) = 0;  % Set TVC angles to zero

    end

end