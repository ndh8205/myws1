%%
function [ U ] = Controller_PID_Argument( X, X_target, dt, params )

    % First loop -> initialize this value
    
    persistent preivous_Pos
    persistent previous_att
    persistent V_err_i
    persistent rate_i
    persistent previous_error_V
    persistent previous_error_r
    persistent rate_i_count
    
    rate_i_count = 1;

    % Initialize persistent variables if they are empty

    if isempty( V_err_i )

        V_err_i = zeros( 2, 1 );
        rate_i = 0;

        previous_error_V = zeros( 2, 1 );
        previous_error_r = 0;

        preivous_Pos = zeros( 2, 1 );
        previous_att = 0;

    end

    rate_now = ( X( 5 ) - previous_att ) / dt; % rate update ( atittude diff. )

    previous_att = X( 5 ); % previous att update
    
    %Parameters set

    Kp_pp = params.Airbearing.Kp_pp;

    Kp_p = params.Airbearing.Kp_p;
    Kp_i = params.Airbearing.Kp_i;
    Kp_d = params.Airbearing.Kp_d;

    Ka_pp = params.Airbearing.Ka_pp;

    Ka_p = params.Airbearing.Ka_p;
    Ka_i = params.Airbearing.Ka_i;
    Ka_d = params.Airbearing.Ka_d;

    % speed_limmit = params.Airbearing.SL1;
    speed_limmit = norm(X_target(3:4));
    disp(speed_limmit);

    rate_limmit = params.Airbearing.SA1;

    % rotation matrix update

    R_B2I = [ cos( X(5) ), -sin( X(5) );
              sin( X(5) ), cos( X(5) ) ];

    R_I2B = R_B2I';

    Vel_now = ( X( 1:2 ) - preivous_Pos ) / dt; % velocity update ( position diff. )
    Vel_now = R_I2B * Vel_now;

    preivous_Pos = X( 1:2 ); % previous Pos update
    
    % Error 
    X_e = X_target - X; % State Error Define

    ang_err = X_e( 5 );

    pos_err = X_e( 1:2 ); % Postion Error - Euler

    % Single P control

    V_cmd = Kp_pp .* pos_err;

    V_cmd = R_I2B * V_cmd; % V_cmd
    
    rate_cmd = Ka_pp * ang_err; % Rate_cmd

    % Limitation part
    if norm( V_cmd ) > speed_limmit

        V_cmd = (V_cmd / norm( V_cmd )) * speed_limmit;

    end  

    V_error = V_cmd - Vel_now; 

    % Limitation part
    if rate_cmd > rate_limmit

        rate_cmd = rate_limmit;

    end  

    rate_err = rate_cmd - rate_now;

    V_err_d = ( V_error - previous_error_V ) / dt; % Vel err. differential
    rate_err_d = ( rate_err - previous_error_r ) / dt; % Rate err. differential;
    
    previous_error_V = V_error;
    previous_error_r = rate_err;

    V_err_i = V_err_i + V_error * dt; % Vel err. integral
    rate_i =  rate_i + rate_err * dt; % Rate err. integral
    rate_i_count = rate_i_count + 1;
    
    if rate_i_count == 20
        rate_i_count = 1;
    end

    % cmd_Vel  - P I D Control

    cmd_Force = ( Kp_p .* V_error ) + ( Kp_i .* V_err_i ) + ( Kp_d .* V_err_d );
    cmd_Torque = ( Ka_p * rate_err ) + ( Ka_i * rate_i ) + ( Ka_d * rate_err_d );

    U = [ cmd_Force; cmd_Torque ];

end