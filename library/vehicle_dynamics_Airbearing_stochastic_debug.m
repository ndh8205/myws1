function [ xdot, F_total ]  = vehicle_dynamics_Airbearing_stochastic_debug( X, U, w, dt, params )
    
    % Parameters set
    
    I = params.Airbearing.I_Z;
    m = params.Airbearing.m;
    L = params.Airbearing.D_ref;
    
    % Thruster Parameters

    T = params.Airbearing.Thrust;

    % State Vector decomposition
    
    r = X( 1:2 ); % Position (X Y) - inertial frame
    V_B = X( 3:4 )';% X(3:4); % Velocity (u v) - body frame
    att = X( 5 ); % Attitude (psi) - inertial frame
    R = X( 6 ); % Angular rate (r) - body frame

    % Control Vector decomposition

    % idle = 49;
    
    Open_Valve = U(1:8);
    RW_tau = cmd2tq_origin( U(9), 49, params, dt );
    % RW_tau = U(9);

    Control_Logic = T .* [   0,  -1,   -1,   0,    0,   1,    1,   0;   % X
                            -1,   0,    0,   1,    1,   0,    0,  -1;   % Y
                            -L,   L,   -L,   L,   -L,   L,   -L,   L ]; % Tau


    final_FT = Control_Logic * Open_Valve;
    Fx = final_FT(1);
    Fy = final_FT(2);
    Tau = final_FT(3) + RW_tau;

    F_total = [ Fx/m; Fy/m; Tau/I ];
    
    % 3 DoF Flat airbearing testbed modeling

    x1_dot =  [ V_B(1) * cos(att) - V_B(2) * sin(att) ;
                V_B(1) * sin(att) + V_B(2) * cos(att) ];  

    x2_dot = [ Fx  / m  - R * V_B(2) ;
               Fy / m  + R * V_B(1) ];
            
    x3_dot = R;

    x4_dot = Tau / I ;

    xdot = [ x1_dot; x2_dot; x3_dot; x4_dot ]; % Xdot state vector

end