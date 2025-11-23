function [ final_cmd, U ]  = Control_Allocator_MPC( U, params )

    persistent pre_Motor_mask;
    
    Motor_mask_init = 4;

    if isempty(pre_Motor_mask)

        pre_Motor_mask = Motor_mask_init;

    end

    Motor_cmd = U( 9 );

    % Scale the delta_w_rw to motor command (0-100, with 40 as idle)
    min_input = -93/40676; % Define minimum input based on your application
    max_input = 0.005; % Define maximum input based on your application

    % min_input = -0.057950; % Define minimum input based on your application
    % max_input = 0.005916; % Define maximum input based on your application


    motor_command = scale_command( Motor_cmd, min_input, max_input, 49 );
    Motor_mask = LIMIT2( motor_command, 0, 98 );

    Relay_mask1 = U(1:8);
    Relay_mask2 = U(1:8);



    pre_Motor_mask = Motor_mask;

   if pre_Motor_mask <= 4 || pre_Motor_mask >= 95 %&& RW_control_rate_limit < abs(rate_now) %

        final_cmd = [ Relay_mask1 ; Motor_mask ];

   else

        final_cmd = [ Relay_mask2 ; Motor_mask ];

   end
    
end