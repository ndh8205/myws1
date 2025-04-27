function [ U ] = Controller_PPID_Argument( X, X_target, dt, params )

    % First loop -> initialize this value
    
    persistent cmd_rate_i 
    persistent cmd_rate_error

    % Initialize persistent variables if they are empty

    if isempty( cmd_rate_error )

        cmd_rate_i = zeros( 3, 1 );
        cmd_rate_error = zeros( 3, 1 );

    end
    
    %Parameters set
    Ka_pp = params.control.Ka_pp;

    Ka_p = params.control.Ka_p;
    Ka_i = params.control.Ka_i;
    Ka_d = params.control.Ka_d;

    % error ( P-controler ) 
    quat_cmd = X_target;
    quat_state = X( 7:10 );

    quat_cmd = quat_cmd / norm( quat_cmd );
    quat_state = quat_state / norm( quat_state );

    inv_quat_state = inv_q( quat_state );

    Angle_quat_e = q2q_mult( inv_quat_state , quat_cmd ); % State Error Define
    Angle_quat_e = Angle_quat_e / norm( Angle_quat_e );

    % sgn_funtion ( P-controler ) 
    SGN = sgn( Angle_quat_e( 1 ) );
    
    % rate command generation ( P-controler ) 
    rate_cmd = ( SGN * 2 * Ka_pp .* Angle_quat_e( 2:4 ) );
     
    % error ( PID-controler ) 
    rate_e = rate_cmd - X( 11:13 ); % State Error Define
    cmd_rate_d = ( rate_e - cmd_rate_error ) / dt; % rate differential
    cmd_rate_i = cmd_rate_i + rate_e * dt; % rate integral
    cmd_rate_error = rate_e;

    % PID control
    U = ( Ka_p .* cmd_rate_error ) + ( Ka_i .* cmd_rate_i ) + ( Ka_d .* cmd_rate_d );

end