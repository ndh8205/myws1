function [ command_Vector, flight_phase ] = generate_commands( i, Sim_Loop_Hz, params )

    % Extract flight phase parameters
    First_burn_time = params.flight.ascent_time;
    No_burn_time = params.flight.descent_start_time;
    
    % landing_start_time = params.flight.landing_start_time;

    % Extract attitude command parameters
    ascent_attitude_1 = params.flight.ascent_attitude_1;
    descent_attitude = params.flight.descent_attitude;
    landing_attitude = params.flight.landing_attitude;

    dt = 1 / Sim_Loop_Hz;
    t = i * dt;

    % Define flight phases based on time
    if t <= First_burn_time

        flight_phase = 'First_burn';

    elseif t <= No_burn_time

        flight_phase = 'No_burn';

    else

        flight_phase = 'landing';

    end

    % Define commands based on flight phase
    switch flight_phase

        case 'First_burn'

            att_target = ascent_attitude_1;

        case 'No_burn'

            att_target = descent_attitude;

        case 'landing'

            att_target = landing_attitude;

    end

    att_target = GetQUAT(att_target(3), att_target(2), att_target(1));
    att_target = att_target / norm(att_target);
    command_Vector = att_target;
end