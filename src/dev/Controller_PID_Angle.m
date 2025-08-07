%% Controller Part

function U = Controller_PID_Angle( X, X_taget, dt, params )
     
    %Parameters set

    % Angle_P gain = [Roll, pitch, Yaw]

    Ka_pp = params.rocket.Ka_pp; %Attitude P

    % error
    
    quat_cmd = X_taget;
    quat_state = X( 7:10 );

    quat_cmd = quat_cmd / norm( quat_cmd );
    quat_state = quat_state / norm( quat_state );

    inv_quat_state = inv_q( quat_state );

    Angle_quat_e = q2q_mult( inv_quat_state , quat_cmd ); % State Error Define
    Angle_quat_e = Angle_quat_e / norm( Angle_quat_e );

    SGN = sgn( Angle_quat_e( 1 ) );
    
    cmd_ang = ( SGN * 2 * Ka_pp .* Angle_quat_e( 2:4 ) );

    U = cmd_ang;
 
end