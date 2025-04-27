function [xdot, Thrust_mass_vec] = vehicle_dynamics_HANul2(X, U, Qk, dt, params, t)
    %% Parameters set
    % Reference dimensions
    S_ref = params.vehicle.S_A_ref;  % Reference area
    D_ref = params.vehicle.D_ref;    % Reference diameter
    L_total = params.vehicle.L;      % Total length
    x_ref = params.vehicle.Lp;       % CP to CG distance
    
    % Environment parameters
    g = params.environment.g;
    rho = params.environment.ro;
    
    % Get viscosity for Reynolds number
    T = 288.15;  % Standard temperature [K]
    mu = 1.458e-6 * T^1.5/(T + 110.4); % Sutherland's law
    
    % Vehicle mass properties
    m_D = params.vehicle.m_D;
    E1M_d = params.vehicle.m_W;
    J = params.vehicle.J;

    % Rocket geometry
    rocket.length = L_total;
    rocket.diameter = D_ref * 2;
    rocket.nose_length = 0.3;  % Set based on actual nose length
    rocket.nose_type = "haack";
    rocket.fin_count = params.fin.num;
    
    % Fin geometry 
    fin.root_chord = params.fin.chord;
    fin.tip_chord = params.fin.chord * 0.6; % Assumed taper ratio
    fin.span = params.fin.span;
    fin.sweep = 30 * pi/180;  % 30 degree sweep
    fin.thickness = 0.003;    % 3mm thickness
    
    % Calculate fin parameters
    fin.area = (fin.root_chord + fin.tip_chord) * fin.span / 2;
    fin.AR = 2 * fin.span^2 / fin.area;
    fin.MAC = (fin.root_chord + fin.tip_chord)/2;
    fin.y_MAC = fin.span/3 * (fin.root_chord + 2*fin.tip_chord)/(fin.root_chord + fin.tip_chord);

    % Motor parameters
    T_1 = params.motor.thrust_func_stage1;
    m1_fuel = params.motor.mass_func_stage1;
    Ti_1 = params.motor.Ti_1;

    % Simulation time handling
    ti = max(1, min(t, Ti_1));
    current_thrust = T_1(ti);
    current_mass = m1_fuel(ti) + E1M_d + m_D;
    F_T = [current_thrust, 0, 0]';
    m = current_mass;
    Thrust_mass_vec = [F_T; m];
    
    %% State Vector decomposition
    pos = X(1:3);    % Position (X Y Z) - inertial frame
    V_B = X(4:6);    % Velocity (u v w) - body frame  
    att = X(7:10);   % Attitude quaternion
    omega = X(11:13); % Angular rates (p q r) - body frame

    % Process Noise Vector decomposition
    Fx_Q = Qk(1); % Translation uncertainty 
    Fy_Q = Qk(2);
    Fz_Q = Qk(3);
    
    Rx_Q = Qk(4); % Rotation uncertainty
    Py_Q = Qk(5);
    Yz_Q = Qk(6);

    %% Calculate aerodynamic parameters
    % Get velocity magnitude and Mach number
    V = norm(V_B);
    Vsound = 340; % Speed of sound [m/s]
    M = V/Vsound;
    
    % Calculate angles
    if V < 1e-8
        alpha = 0;
        beta = 0;
    else
        V_Bi = V_B / V;
        beta = asin(V_Bi(2));
        alpha = atan2(V_Bi(3), V_Bi(1));
    end
    alpha_tot = sqrt(alpha^2 + beta^2);

    % Reynolds number
    Re = rho * V * L_total / mu;
    
    %% Aerodynamic Coefficients
    
    % Normal Force Coefficients
    % Body contribution
    CNa_body = 2 * ((L_total - rocket.nose_length) * rocket.diameter) / S_ref;

    % Nose contribution
    if rocket.nose_type == "conical"
        CNa_nose = 2;
    else % haack or ogive
        CNa_nose = 2.4;
    end

    % Fin contribution with interference
    Kfb = 1 + rocket.diameter/(2 * fin.span);
    CNa_fin = Kfb * (4 * rocket.fin_count * (fin.AR / (2 + sqrt(fin.AR^2 + 4)))) * (fin.area/S_ref);
    
    % Total normal force
    CNa_total = CNa_nose + CNa_body + CNa_fin;
    
    % Prandtl-Glauert correction
    if M < 1
        beta = sqrt(1 - M^2);
        CNa_total = CNa_total/beta;
    else
        beta = sqrt(M^2 - 1);
    end

    % Skin friction drag
    if Re < 1e4
        Cf = 1.48e-2;
    elseif Re < 5e5  % Transition Re
        Cf = 1/((1.50*log(Re) - 5.6)^2);
    else
        Rs = 60e-6;  % Surface roughness [m]
        Cf = 0.032*(Rs/L_total)^0.2;
    end

    % Compressibility correction
    if M < 1
        Cf_c = Cf * (1 - 0.1*M^2);
    else
        Cf_c = Cf/(1 + 0.15*M^2)^0.58;
    end

    % Base drag
    if M < 1
        CD_base = 0.12 + 0.13*M^2;
    else
        CD_base = 0.25/M;
    end

    % Wetted areas
    A_wet_body = pi * rocket.diameter * L_total;
    A_wet_fins = 2 * rocket.fin_count * fin.area;
    A_base = pi * (rocket.diameter/2)^2;

    % Total axial force coefficient including skin friction
    fineness = L_total/rocket.diameter;
    CA = Cf_c * ((1 + 1/(2*fineness))*A_wet_body + (1 + 2*fin.thickness/fin.MAC)*A_wet_fins)/S_ref + CD_base * A_base/S_ref;

    % Roll coefficients for canted fins
    fin_cant = params.fin.max_angle;  % Fin cant angle
    Cl_force = rocket.fin_count * (fin.y_MAC + rocket.diameter/2) * CNa_fin * fin_cant/rocket.diameter;
    
    % Roll damping (simplified)
    roll_damp = -rocket.fin_count * CNa_fin * fin.span^2/(4 * S_ref * L_total);
    Cl_damp = roll_damp * omega(1) * D_ref/(2*V);
    
    % Total coefficients
    CN = CNa_total * alpha_tot;
    CY = CNa_total * beta;
    Cl = Cl_force + Cl_damp;
    Cm = -CNa_total * alpha * x_ref/D_ref;
    Cn = CNa_total * beta * x_ref/D_ref;

    %% Aerodynamic Forces and Moments
    q = 0.5 * rho * V^2;  % Dynamic pressure
    
    F_A = q * S_ref * [-CA; CY; -CN];
    M_A = q * S_ref * D_ref * [Cl; Cm; Cn];

    %% Control Vector decomposition
    Open_Valve = U; % RCS valve states (8 thrusters)
    
     % Control Logic Matrix
                          %T1       %T2       %T3       %T4       %T5    %T6      %T7       %T8   
    Control_Logic = [       0,        0,       0,        0,       0,        0,       0,        0;   % Fx
                            0,       -1,      -1,        0,       0,        1,       1,        0;   % Fy
                           -1,        0,       0,        1,       1,        0,       0,       -1;   % Fz
                        D_RCS,   -D_RCS,   D_RCS,   -D_RCS,   D_RCS,   -D_RCS,   D_RCS,   -D_RCS;   % Roll
                       -D_RCS,        0,       0,   -D_RCS,   D_RCS,        0,       0,    D_RCS;   % Pitch
                            0,    -D_RCS,  D_RCS,        0,       0,   -D_RCS,   D_RCS,        0 ]; % Yaw


    final_FT_RCS = T_RCS * Control_Logic * Open_Valve;

    Force_RCS = final_FT_RCS(1:3);
    Torque_RCS = final_FT_RCS(4:6);

    % Get rotation matrices
    R_B2I_q = GetDCM_QUAT(att);
    R_I2B_q = R_B2I_q';

    %% Sum Forces and Moments
    F_G = R_I2B_q * m*g;    % Gravity in body frame
    F_total = F_A + F_G + F_T + Force_RCS + [Fx_Q; Fy_Q; Fz_Q];
    M_total = M_A + Torque_RCS + [Rx_Q; Py_Q; Yz_Q];

    %% 6-DOF Integration
    att = att / norm(att);  % Normalize quaternion

    x1_dot = R_B2I_q * V_B;
    x2_dot = (F_total / m) - cross(omega, V_B);
    x3_dot = Derivative_Quat(att, omega);
    x4_dot = inv(J) * (M_total - cross(omega, J*omega));

    xdot = [x1_dot; x2_dot; x3_dot; x4_dot];

end