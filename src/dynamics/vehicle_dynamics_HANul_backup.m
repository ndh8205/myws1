%% Dynamics Part

function [xdot, Thrust_mass_vec] = vehicle_dynamics_HANul_backup( X, U, Qk, dt,params, t )
    %% Parameters set
    S_A_ref = params.vehicle.S_A_ref;
    D_ref = params.vehicle.D_ref;
    J = params.vehicle.J;
    x_ref = params.vehicle.Lp;

    C_A = params.aero.C_A;
    C_S_beta = params.aero.C_S_beta;
    C_N_alpha = params.aero.C_N_alpha;
    C_l_p = params.aero.C_l_p;
    C_m_q = params.aero.C_m_q;
    C_m_alpha = params.aero.C_m_alpha;
    C_n_r = params.aero.C_n_r;
    C_n_beta = params.aero.C_n_beta;

    g = params.environment.g;
    ro = params.environment.ro;
    
    m_D = params.vehicle.m_D;
    E1M_d = params.vehicle.m_W;
    T_1 = params.motor.thrust_func_stage1;
    m1_fuel = params.motor.mass_func_stage1;
    Ti_1 = params.motor.Ti_1;

    ground_level = 0;

    % RCS Parameters
    L_RCS = params.vehicle.Lrcs;
    T_RCS = params.vehicle.RCS_T; % Thrust per RCS thruster
    D_RCS = params.vehicle.r_ref; % Lever arm distance for RCS thrusters

    ti = max(1, min(t, Ti_1));

    current_thrust = T_1(ti);
    current_mass = m1_fuel(ti) + E1M_d + m_D;

    F_T = [current_thrust, 0, 0]';
    m = current_mass;

    Thrust_mass_vec = [F_T; m];
    
    %% State Vector decomposition
    pos = X(1:3); % Position (X Y Z) - inertial frame
    V_B = X(4:6); % Velocity (u v w) - body frame
    att = X(7:10); % Attitude (q0 q1 q2 q3) - inertial frame
    omega = X(11:13); % Angular rate (p q r) - body frame

    % Process Noise Vector decomposition
    Fx_Q = Qk(1); % Translation uncertainty
    Fy_Q = Qk(2);
    Fz_Q = Qk(3);

    Rx_Q = Qk(4); % Rotation uncertainty
    Py_Q = Qk(5);
    Yz_Q = Qk(6);

    %% Control Vector decomposition
    Open_Valve = U; % RCS valve states ( 8 thrusters )
    
    % Control Logic Matrix
                          %T1       %T2       %T3       %T4       %T5    %T6      %T7       %T8   
    Control_Logic = [       0,        0,       0,        0,       0,        0,       0,        0;   % Fx
                            0,       -1,      -1,        0,       0,        1,       1,        0;   % Fy
                           -1,        0,       0,        1,       1,        0,       0,       -1;   % Fz
                        D_RCS,   -D_RCS,   D_RCS,   -D_RCS,   D_RCS,   -D_RCS,   D_RCS,   -D_RCS;   % Roll
                       -L_RCS,        0,       0,   -L_RCS,   L_RCS,        0,       0,    L_RCS;   % Pitch
                            0,    -L_RCS,  L_RCS,        0,       0,   -L_RCS,   L_RCS,        0 ]; % Yaw

    final_FT_RCS = T_RCS * Control_Logic * Open_Valve;

    Fx_RCS = final_FT_RCS(1);
    Fy_RCS = final_FT_RCS(2);
    Fz_RCS = final_FT_RCS(3);
    Roll_RCS = final_FT_RCS(4);
    Pitch_RCS = final_FT_RCS(5);
    Yaw_RCS = final_FT_RCS(6);

    Force_RCS = [ Fx_RCS; Fy_RCS; Fz_RCS ];
    Torque_RCS = [ Roll_RCS; Pitch_RCS; Yaw_RCS ];

    % Rotation Matrix Define quaternion
    
    R_B2I_q = GetDCM_QUAT( att );
    R_I2B_q = R_B2I_q';
    
    %% Aerodynamics
    q_aero = 0.5 * ro * norm(V_B)^2; % dynamic pressure
    Damp_M = D_ref / 2 * norm(V_B); 

    % V_B angle Define
    
    if norm(V_B) < 1e-8
        alpha = 0; % Angle of Attack
        beta = 0;  % Side slip
    else
        V_Bi = V_B / norm(V_B);
        beta = asin(V_Bi(2));
        alpha = atan2(V_Bi(3), V_Bi(1));
        
    end

    C_A_total = -C_A * q_aero * S_A_ref; % Axial Force
    C_S_total = C_S_beta * beta * q_aero * S_A_ref; % Side Force
    C_N_total = C_N_alpha * alpha * q_aero * S_A_ref; % Normal Force  

    C_l = (Damp_M * C_l_p * omega(1)) * q_aero * S_A_ref * D_ref; % Aero Roll Moment
    C_m = (Damp_M * C_m_q * omega(2) + C_m_alpha * alpha) * q_aero * S_A_ref * D_ref; % Aero Pitch Moment
    C_n = (Damp_M * C_n_r * omega(3) + C_n_beta * beta) * q_aero * S_A_ref * D_ref; % Aero Yaw Moment

    Cp2CG = [ x_ref; 0; 0 ]; % Center of Pressure to Center of Gravity

    F_A = [C_A_total; C_S_total; C_N_total]; % Force Vector (Aero) - body frame
    M_A_Mrp = [ C_l; C_m; C_n ]; % Aero Moments

    %% gravitaional & Thurst   
    F_G = R_I2B_q * m*g;    % Force Vector (Gravitation) - body frame
    
    % Thrust Force Vector - body frame
    F_T_body = [current_thrust; 0; 0]; % Assuming thrust is aligned with the X-axis

    % Aero + Gravity + Thrust + RCS
    F_total = F_A + F_G + F_T_body + Force_RCS + [ Fx_Q; Fy_Q; Fz_Q ];

    % Aero + Thrust + RCS
    Torque_A_body = M_A_Mrp + cross(F_A, Cp2CG); % Aero Moments in body frame
    M_T = [0; 0; 0]; % Thrust bias

    M_total = Torque_A_body + Torque_RCS + M_T + [ Rx_Q; Py_Q; Yz_Q ];

    % if -pos(3) <= ground_level
    % 
    %     pos(3) = ground_level;
    %     V_B = zeros(3,1);
    %     F_total = [0; 0; 0];
    %     M_total = [0; 0; 0];
    % 
    % end

    att = att / norm(att);

    % 6 DOF Rocket Modeling
 
    x1_dot = R_B2I_q * V_B; % inertial frame

    x2_dot = (F_total / m) - cross(omega, V_B); % body frame

    % if pos(3) <= ground_level
    %     x1_dot(3) = 0;
    %     x2_dot(3) = 0;
    % end

    x3_dot = Derivative_Quat(att, omega); % inertial frame

    x4_dot = inv(J) * ( M_total - cross( omega, J*omega ) ); % body frame

    xdot = [x1_dot; x2_dot; x3_dot; x4_dot]; % Xdot state vector

end


