function [ command_Vector ] = generate_commands_HANul( i, Sim_Loop_Hz, params )

    att_target = deg2rad( [30, 89.999, 0] );

    att_target = GetQUAT(att_target(3), att_target(2), att_target(1));
    att_target = att_target / norm(att_target);
    command_Vector = att_target;
end