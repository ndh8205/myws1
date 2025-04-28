function [xdot, Thrust_mass_vec] = vehicle_dynamics_HANul(X, U, Qk, dt, params, t)
    % Parameters set
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
    mach_crit = params.aero.mach_crit;
    mach_super = params.aero.mach_super;

    g = params.environment.g;
    
    m_D = params.vehicle.m_D;
    T_1 = params.motor.thrust_func_stage1;
    m1_fuel = params.motor.mass_func_stage1;
    Ti_1 = params.motor.Ti_1;

    % RCS Parameters
    L_RCS = params.vehicle.Lrcs;
    T_RCS = params.vehicle.RCS_T;
    D_RCS = params.vehicle.r_ref;

    % Get atmospheric properties using ISA model
    [Temp, a, Pa, rho] = atmosisa(-X(3));
    ro = rho;

    % Motor parameters
    ti = max(1, min(t, Ti_1));
    current_thrust = T_1(ti);
    current_mass = m1_fuel(ti) + m_D;
    
    F_T = [current_thrust, 0, 0]';
    m = current_mass;
    Thrust_mass_vec = [F_T; m];

    % State Vector decomposition
    pos = X(1:3);      % Position (X Y Z) - inertial frame
    V_B = X(4:6);      % Velocity (u v w) - body frame
    att = X(7:10);     % Attitude (q0 q1 q2 q3) - inertial frame
    omega = X(11:13);  % Angular rate (p q r) - body frame

    % Process Noise Vector decomposition
    Fx_Q = Qk(1);    % Translation uncertainty
    Fy_Q = Qk(2);
    Fz_Q = Qk(3);
    Rx_Q = Qk(4);    % Rotation uncertainty
    Py_Q = Qk(5);
    Yz_Q = Qk(6);

    % Add wind effects
    persistent wind_time wind_speed
    if isempty(wind_time) || t >= wind_time + 0.05  % 20Hz update
        wind_time = t;
        % Generate wind using pink noise
        wind_turb = randn() * params.wind.v_avg * params.wind.turb_intensity;
        wind_speed = params.wind.v_avg + wind_turb;
    end

    % Get rotation matrices
    R_B2I_q = GetDCM_QUAT(att);
    R_I2B_q = R_B2I_q';

    % Transform wind to body coordinates
    wind_I = [wind_speed * cos(params.wind.direction);
              wind_speed * sin(params.wind.direction);
              0];
    V_wind_B = R_I2B_q * wind_I;
    V_B_aero = V_B - V_wind_B;  % Relative velocity for aero calculations

    % Calculate Mach number and update coefficients
    M = norm(V_B_aero)/a;
    
    % Update coefficients based on flight regime
    if M < mach_crit
        % Subsonic - Prandtl-Glauert correction
        beta_PG = sqrt(1 - M^2);
        C_N_alpha = C_N_alpha / beta_PG;
        C_S_beta = C_S_beta / beta_PG;
        
    elseif M <= mach_super
        % Transonic - interpolation
        w = (M - mach_crit)/(mach_super - mach_crit);
        C_N_alpha = C_N_alpha * (1 + w*0.5);
        C_S_beta = C_S_beta * (1 + w*0.5);
        
    else
        % Supersonic
        C_N_alpha = C_N_alpha * 2/sqrt(M^2 - 1);
        C_S_beta = C_S_beta * 2/sqrt(M^2 - 1);
    end
    
    % Update drag coefficient with wave drag
    if M > mach_crit
        if M <= mach_super
            % Transonic drag rise
            wave_drag = 5*(M - mach_crit)^2;
        else
            % Supersonic wave drag
            wave_drag = 1/(sqrt(M^2 - 1));
        end
        C_A = C_A * (1 + wave_drag);
    else
        C_A = C_A;
    end

    % Control Vector decomposition
    Open_Valve = U; % RCS valve states (8 thrusters)
    
    % Control Logic Matrix
                          %T1       %T2       %T3       %T4       %T5       %T6       %T7       %T8   
    Control_Logic = [       0,        0,        0,        0,        0,        0,        0,        0;    % Fx
                           0,       -1,       -1,        0,        0,        1,        1,        0;    % Fy
                          -1,        0,        0,        1,        1,        0,        0,       -1;    % Fz
                       D_RCS,   -D_RCS,    D_RCS,   -D_RCS,    D_RCS,   -D_RCS,    D_RCS,   -D_RCS;  % Roll
                      -L_RCS,        0,        0,   -L_RCS,    L_RCS,        0,        0,    L_RCS;  % Pitch
                           0,   -L_RCS,    L_RCS,        0,        0,   -L_RCS,    L_RCS,        0 ]; % Yaw

    final_FT_RCS = T_RCS * Control_Logic * Open_Valve;

    Force_RCS = final_FT_RCS(1:3);
    Torque_RCS = final_FT_RCS(4:6);

    % V_B angle Define
    if norm(V_B_aero) < 1e-8
        alpha = 0;  % Angle of Attack
        beta = 0;   % Side slip
    else
        V_Bi = V_B_aero / norm(V_B_aero);
        beta = asin(V_Bi(2));
        alpha = atan2(V_Bi(3), V_Bi(1));
    end

    q_aero = 0.5 * ro * norm(V_B_aero)^2;  % dynamic pressure
    Damp_M = D_ref / 2 * norm(V_B_aero);

    C_A_total = -C_A * q_aero * S_A_ref;      % Axial Force
    C_S_total = C_S_beta * beta * q_aero * S_A_ref;    % Side Force
    C_N_total = C_N_alpha * alpha * q_aero * S_A_ref;  % Normal Force

    C_l = (Damp_M * C_l_p * omega(1)) * q_aero * S_A_ref * D_ref;            % Aero Roll Moment
    C_m = (Damp_M * C_m_q * omega(2) + C_m_alpha * alpha) * q_aero * S_A_ref * D_ref;  % Aero Pitch Moment
    C_n = (Damp_M * C_n_r * omega(3) + C_n_beta * beta) * q_aero * S_A_ref * D_ref;    % Aero Yaw Moment

    Cp2CG = [x_ref; 0; 0];  % Center of Pressure to Center of Gravity

    F_A = [C_A_total; C_S_total; C_N_total];  % Force Vector (Aero) - body frame
    M_A_Mrp = [C_l; C_m; C_n];  % Aero Moments

    % Gravitational Force - body frame
    F_G = R_I2B_q * (m*g);
    
    % Thrust Force Vector - body frame
    F_T_body = [current_thrust; 0; 0];

    % Total Forces and Moments
    F_total = F_A + F_G + F_T_body + Force_RCS + [Fx_Q; Fy_Q; Fz_Q];
    
    Torque_A_body = M_A_Mrp + cross(F_A, Cp2CG);
    M_T = [0; 0; 0];  % Thrust bias
    M_total = Torque_A_body + Torque_RCS + M_T + [Rx_Q; Py_Q; Yz_Q];

    % State Equations
    att = att / norm(att);  % Normalize quaternion

    x1_dot = R_B2I_q * V_B;  % Position derivative - inertial frame
    x2_dot = (F_total / m) - cross(omega, V_B);  % Velocity derivative - body frame
    x3_dot = Derivative_Quat(att, omega);  % Attitude derivative - inertial frame
    x4_dot = inv(J) * (M_total - cross(omega, J*omega));  % Angular rate derivative - body frame

    xdot = [x1_dot; x2_dot; x3_dot; x4_dot];
end