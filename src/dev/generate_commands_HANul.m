function [ command_Vector ] = generate_commands_HANul( i, Sim_Loop_Hz, params )

    current_time = (i-1) * 1/Sim_Loop_Hz;
    command_Vector = zeros(5,1);
    
    if current_time < 0

        main_engine_sig = 0;
        command_Vector(1) = main_engine_sig;

        att_target = deg2rad( [0, 90, 0] );
        att_target = GetQUAT(att_target(3), att_target(2), att_target(1));
        att_target = att_target / norm(att_target);
        command_Vector(2:5) = att_target';

    else

        main_engine_sig = 1;
        command_Vector(1) = main_engine_sig;

        att_target = deg2rad( [90, 89, 0] );
        att_target = GetQUAT(att_target(3), att_target(2), att_target(1));
        att_target = att_target / norm(att_target);
        command_Vector(2:5) = att_target';

    end
end