function [ final_cmd, U ]  = Control_Allocator_RCS( U, params )

    Jyy = params.vehicle.J_Y; % [kgmm^2]
    Jxx = params.vehicle.J_X; % [kgmm^2]
    d = params.vehicle.r_ref; % [m] (CG to Thruster - Y&Z) 
    l = params.vehicle.Lrcs;  % [m] (CG to Thruster - X) 
    m = params.vehicle.m_D; % [kg] (Airbearing mass)
    T = params.vehicle.RCS_T; % [N] (Mean Thrust Force)

    M = T/m;
    R_RCS = (T*d)/Jxx;
    L_RCS = (T*l)/Jyy;

                  %T1       %T2      %T3       %T4      %T5       %T6      %T7       %T8   
    A_total_cmd = [       0,        0,       0,        0,       0,        0,       0,        0;       % Fx
                          0,       -M,      -M,        0,       0,        M,       M,        0;       % Fy
                         -M,        0,       0,        M,       M,        0,       0,       -M;       % Fz
                      R_RCS,   -R_RCS,   R_RCS,   -R_RCS,   R_RCS,   -R_RCS,   R_RCS,   -R_RCS;       % Roll
                     -L_RCS,        0,       0,   -L_RCS,   L_RCS,        0,       0,    L_RCS;       % Pitch
                          0,    -L_RCS,  L_RCS,        0,       0,   -L_RCS,   L_RCS,        0 ];     % Yaw

    RPY = A_total_cmd(4:6, :);

    U = pinv( RPY ) * U;

    for i = 1 : 8

        U(i) = LIMIT2( U(i), -20, 20 );

    end

    final_cmd = U( 1:8 ) > mean(U);

end