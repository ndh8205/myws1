%% Dynamics Part

function [xdot, Thrust_mass] = vehicle_dynamics(X, U, params, t)
    
    % Parameters set

    S_ref = params.rocket.S_ref;
    D_ref = params.rocket.D_ref;
    I = params.rocket.I; 
    g = params.environment.g;
    ro = params.environment.ro;
    C_A = params.aero.C_A;
    C_S_beta = params.aero.C_S_beta;
    C_N_alpha = params.aero.C_N_alpha;
    C_l_p = params.aero.C_l_p;
    C_m_q = params.aero.C_m_q;
    C_m_alpha = params.aero.C_m_alpha;
    C_n_r = params.aero.C_n_r;
    C_n_beta = params.aero.C_n_beta;
    x_ref = params.rocket.x_ref;
    x_Nozzle = params.rocket.x_Nozzle;
    dt = 1/ params.Simulation.Sim_Loop_Hz;

    m_f = params.rocket.m_f;
    
    E1M_d = params.motor.E1M_d;
    E2M_d = params.motor.E2M_d;

    T_1 = params.motor.thrust_func_stage1;
    T_2 = params.motor.thrust_func_stage2;

    m1_fuel = params.motor.mass_func_stage1;
    m2_fuel = params.motor.mass_func_stage2;

    Ti_1 = params.motor.Ti_1;
    Ti_2 = params.motor.Ti_2;

    T_LB = params.motor.stage2_ignition_time * 1/dt;
    land_sig = params.motor.land_sig;
    impact_checker_d = 0;

    if t >= T_LB && land_sig == 1 % 2단 로켓

        ti = max( 1, min( t - T_LB, Ti_2 ) );

        current_thrust = T_2(ti);
        current_mass = m2_fuel(ti) + E2M_d + m_f;

    else % 1단 로켓

        ti = max( 1, min( t, Ti_1 )  );

        current_thrust = T_1(ti);
        current_mass = m1_fuel(ti) + m2_fuel(1) + E1M_d + E2M_d  +  m_f;

    end


    F_T = [ current_thrust, 0, 0 ]';
    m = current_mass;

    Thrust_mass = [ F_T; m ];

    
    % StateVoctor decomposition
    
    pos = X( 1:3 ); % Postion (X Y Z) - inertial frame
    V_B = X( 4:6 ); % Velocity (u v w) - body frame
    att = X( 7:10 ); % Atittude (q0 q1 q2 q3) - inertial frame
    omega = X( 11:13 ); % Angular rate (p q r) - body frame

    % U = [0 0 0]'; % only TEST code (do not activate)

    % Extract control inputs

    fin_def = U(1:4);
    ga1 = U(5);
    ga2 = U(6);
    del_omega = U(7);

    % Rotation Marix Define quaternion

    R_B2I_q = GetDCM_QUAT( att ) ;
    R_I2B_q = R_B2I_q';


    % V_B angle Define

    if norm( V_B ) < 0.0000001

        alpha = 0; % Angle of Attack
        beta = 0; % Side slip

    else

        V_Bi = V_B / norm( V_B );
        alpha =  atan2( V_Bi( 3 ), V_Bi( 1 ) );
        beta = asin( V_Bi( 2 )/ norm( V_Bi )  ); % Side slip
        

    end

    % force & Moment
    q_aero = 0.5 * ro * norm( V_B )^2; %dynamic presure
    Damp_M = D_ref /2 * norm( V_B );
      
    C_A =  C_A * q_aero * S_ref; % Axial Force
    C_S =  C_S_beta * beta * q_aero * S_ref ; % Side Force
    C_N =  C_N_alpha * alpha * q_aero * S_ref ; % Normal Force

    C_l = ( Damp_M * C_l_p * omega( 1 ) ) * q_aero * S_ref * D_ref; % Aero Roll Moment
    C_m = ( Damp_M * C_m_q * omega( 2 ) + C_m_alpha * alpha ) * q_aero * S_ref * D_ref; % Aero Pitch Moment
    C_n = ( Damp_M * C_n_r * omega( 3 ) + C_n_beta * beta ) * q_aero * S_ref * D_ref; % Aero Yaw Moment

    Cp2CG = [ x_ref , 0 , 0 ]';

    F_A = [ -C_A, -C_S, -C_N ]'; % Force Vector (Aero) - body frame
    F_G = R_I2B_q * g * m ; % Force Vector (Gravitation) - body frame
    F_T = [ norm( F_T ) * cos( ga1 ) * cos( ga2 ) ; norm( F_T ) * cos( ga1 ) * sin( ga2 ) ; -norm( F_T ) * sin( ga1 ) ];% Thrust Force Vector - body frame

    M_A_Mrp = [ C_l, C_m, C_n ]'; % Moment Vector (Aero) - MRP frame
    M_A_body = M_A_Mrp + cross( F_A , Cp2CG ); % Moment Vector (Aero) - body frame
    % M_fin = calculate_fin_moment(alpha, beta, fin_deflection, V_B, params); % fin Moment Vector (Aero) - body frame
    % M_A_body = M_A_body + M_fin;  %body effect & fin effect Argument (Aero) - body frame

    M_T = [ 0 ; -norm( F_T ) * sin( ga1 ) * abs(x_Nozzle) ; -norm( F_T ) * cos( ga1 ) * sin( ga2 ) * abs(x_Nozzle) ]; % Thrust Moment Vector - body frame 
    M_RCS = [ del_omega; 0; 0 ]; % Reaction control Vector - body frame 

    % M_A_body = [0;0;0]; % only TEST code (do not activate)

    ground_level = 0;

    if pos(3) > ground_level && impact_checker_d == 0   

        pos(3) = ground_level;
        V_B = zeros(3,1);
        F_G = zeros(3,1);
        F_A = zeros(3,1);
        F_T = zeros(3,1);

    end

    att = att / norm(att);

    % 6 DOF Rocket Modeling
 
    x1_dot = R_B2I_q * V_B; % inertial frame

    x2_dot = ( ( F_A + F_G + F_T ) / m ) - cross( omega, V_B ); % body frame

    if pos(3) > ground_level

    x1_dot(3) = 0;
    x2_dot(3) = 0;
    
    end

    
    x3_dot = Derivative_Quat( att, omega ); % inertial frame

    x4_dot = inv( I )*( ( M_A_body + M_T + M_RCS ) - cross( omega, I * omega ) ); % body frame

    xdot = [ x1_dot; x2_dot; x3_dot; x4_dot ]; % Xdot state vector

end