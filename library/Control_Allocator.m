function [ final_cmd, U ]  = Control_Allocator( U, params )

    persistent pre_Motor_mask;
    
    Motor_mask_init = 4;

    I = params.Airbearing.I_Z; % [kgmm^2]
    d = params.Airbearing.D_ref; % [mm] (CG to Thruster) 
    m = params.Airbearing.m; % [kg] (Airbearing mass)
    T = params.Airbearing.Thrust; % [N] (Mean Thrust Force)

    if isempty(pre_Motor_mask)

        pre_Motor_mask = Motor_mask_init;

    end

  % A_cmd =    T1   T2   T3   T4   T5   T6   T7   T8   RW    ----> 9 x 1
    A_cmd1 = [       0,   -T/m,   -T/m,       0,       0,    T/m,     T/m,      0,     0 ;  % X
                 -T/m,      0,      0,     T/m,     T/m,      0,       0,   -T/m,     0 ;  % Y
               -T*d/I,  T*d/I,  -T*d/I,  T*d/I,  -T*d/I,  T*d/I,  -T*d/I,  T*d/I,   1/I ]; % Psi

    A_cmd2 = [       0,   -T/m,   -T/m,       0,       0,    T/m,     T/m,      0,     0 ;  % X
                 -T/m,      0,      0,     T/m,     T/m,      0,       0,   -T/m,     0 ;  % Y
               -T*d/I,  T*d/I,  -T*d/I,  T*d/I,  -T*d/I,  T*d/I,  -T*d/I,  T*d/I,   0 ]; % Psi


    U1 = pinv( A_cmd1 ) * U; % A * ( A * A^T )^-1 moore penrose inverse matrix
    U2 = pinv( A_cmd2 ) * U; % A * ( A * A^T )^-1 moore penrose inverse matrix

    for i = 1 : 8

        % U1(i) = LIMIT2( U1(i), -20, 20 );
        U2(i) = LIMIT2( U2(i), -20, 20 );

    end

    Relay_mask1 = U1( 1:8 ) > zeros( 8 ,1 );
    Relay_mask2 = U2( 1:8 ) > zeros( 8 ,1 );


    Motor_cmd = U1( 9 );

    % Scale the delta_w_rw to motor command (0-100, with 40 as idle)
    min_input = -0.001; % Define minimum input based on your application
    max_input = 0.001; % Define maximum input based on your application

    motor_command = scale_command( Motor_cmd, min_input, max_input, 49 );
    Motor_mask = LIMIT2( motor_command, 0, 98 );


    pre_Motor_mask = Motor_mask;

   if pre_Motor_mask <= 4 || pre_Motor_mask >= 95 %&& RW_control_rate_limit < abs(rate_now) %

        final_cmd = [ Relay_mask1 ; Motor_mask ];

   else

        final_cmd = [ Relay_mask2 ; Motor_mask ];

   end
    
end