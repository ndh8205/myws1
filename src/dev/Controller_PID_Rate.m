function U = Controller_PID_Rate( X, X_target, dt, params )
 
    % First loop -> initialize this value

    persistent cmd_rate_i cmd_rate_error
    
 
    % Initialize persistent variables if they are empty

    if isempty(cmd_rate_i)

        cmd_rate_i = zeros( 3, 1 );
        cmd_rate_error = zeros( 3, 1 );

    end
     
    %Parameters set 
 
    % Rate_PID = [Roll, pitch, Yaw]

    Ka_p = params.rocket.Ka_p; %Attitude P 
    Ka_i = params.rocket.Ka_i; %Attitude I
    Ka_d = params.rocket.Ka_d; %Attitude D

    % Error calculation

    X_e = X_target - X( 11:13 ); % State Error Define
    % cmd_rate_d = ( X_e - cmd_rate_error ) / dt; % rate differential
    cmd_rate_d = X_target /  dt;
    cmd_rate_i = cmd_rate_i + cmd_rate_error * dt; % rate integral
    cmd_rate_error = X_e;
    % PID control
    U = ( Ka_p .* cmd_rate_error ) + ( Ka_i .* cmd_rate_i ) + ( Ka_d .* cmd_rate_d );

end