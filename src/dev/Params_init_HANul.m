function params = Params_init_HANul()
    %% Environment Parameters

    % Basic Parameters
    params.environment.g = [0, 0, 9.81]'; % [m/s^2]
    params.environment.ro = 1.666; % [kg/m^3]
    params.environment.ro_2 = 1.24; % [kg/m^3]
    
    % Spaceport America Environment Parameters (June 19-22)
    params.environment.latitude = 32.93952; % [degrees]
    params.environment.longitude = -106.92006; % [degrees]
    params.environment.altitude = 1401; % [m] Elevation
    
    % Average Weather Conditions (based on 2023-2024 data)
    params.environment.wind_speed = 6.0; % [m/s] (average from 2023-2024)
    params.environment.pressure = 1025; % [hPa] (2024 average)
    params.environment.temperature = 29; % [°C] (predicted average)
    params.environment.temp_range = [21, 35]; % [°C] (min, max based on historical data)

    %% PID Parameters
    % Angle_PPID gain = [Roll, pitch, Yaw]
    params.control.Ka_pp = [0, 0, 0]'; %Attitude P
    params.control.Ka_p = [0, 0, 0]'; % Rate P
    params.control.Ka_i = [0, 0, 0]'; % Rate I
    params.control.Ka_d = [0, 0, 0]'; % Rate D

    %% Rocket Parameters
    params.vehicle.J_X = 0.061; % [kgm^2]
    params.vehicle.J_Y = 13.4; % [kgm^2]
    params.vehicle.J_Z = 13.4; % [kgm^2]
    params.vehicle.S_W_ref = 1.1068; % [m^2] % surface area
    params.vehicle.S_A_ref = 0.0132; % [m^2] % diameter area
    params.vehicle.m_D = 20.014; % [kg] (Dry mass)
    params.vehicle.m_W_const = 23.647; % [kg] (Wet mass)
    params.vehicle.D_ref = 0.13; % [m] % Rocket diameter
    params.vehicle.r_ref = 0.065; % [m] % Rocket radius
    params.vehicle.Lp = -0.32; % [m] % Center of Presure distance
    params.vehicle.Lt = -1.06; % [m] % Center to Nozzle distance
    params.vehicle.Lrcs = 1; % [m] % Center to RCS distance
    params.vehicle.L = 2.71; % [m] % Rocket length
    params.vehicle.RCS_T = 1; % [N] % Reaction control system thruster

    % Store inertia tensor directly as a matrix
    params.vehicle.J = [params.vehicle.J_X, 0, 0;
                       0, params.vehicle.J_Y, 0;
                       0, 0, params.vehicle.J_Z];

    %% Motor Parameters
    % Read CSV file
    opts = detectImportOptions('AeroTech_M2400T.csv', 'VariableNamingRule', 'preserve');
    data_stage1 = readtable('AeroTech_M2400T.csv', opts);

    % Engine dry mass
    params.motor.E1M_d = data_stage1.("Mass(g)")(end)/1000; % g to kg

    % Interpolation setup - 1000Hz
    time_stage1 = data_stage1.("Time(s)");
    total_time = max(time_stage1) - min(time_stage1);
    sampling_rate = 400;
    interp_point_1 = round(total_time * sampling_rate) + 1;
    new_time_stage1 = linspace(min(time_stage1), max(time_stage1), interp_point_1);

    % Interpolation
    interp_mass_stage1 = interp1(time_stage1, data_stage1.("Mass(g)")/1000, new_time_stage1, 'linear');
    interp_thrust_stage1 = interp1(time_stage1, data_stage1.("Thrust(N)"), new_time_stage1, 'linear');
    
    params.motor.thrust_func_stage1 = interp_thrust_stage1;
    params.motor.mass_func_stage1 = interp_mass_stage1;
    params.vehicle.m_W = params.motor.mass_func_stage1(1);
    params.motor.Ti_1 = interp_point_1;

    %% Sensor/Noise Parameters

    params.Qnoise.s_px = 0; % lon
    params.Qnoise.s_py = 0; % lat
    params.Qnoise.s_pz = 0; % alt

    params.Qnoise.s_u = 0.01;
    params.Qnoise.s_v = 0.01;
    params.Qnoise.s_w = 0.01;

    params.Qnoise.s_thx = deg2rad(0);
    params.Qnoise.s_thy = deg2rad(0);
    params.Qnoise.s_thz = deg2rad(0);

    params.Qnoise.s_wx = deg2rad(0.01);
    params.Qnoise.s_wy = deg2rad(0.01);
    params.Qnoise.s_wz = deg2rad(0.01);

    % Rq_p = deg2rad(0.01);
    % Rq_q = deg2rad(0.01);
    % Rq_r = deg2rad(0.01);
    % 
    % Rq_ax = 0.01;
    % Rq_ay = 0.01;
    % Rq_az = 0.01;
    % 
    % Rq_mx = 0.01;
    % Rq_my = 0.01;
    % Rq_mz = 0.01;
    % 
    % Rq_baro = 0.01;

    % --- 추가로 10개 바이어스 잡음 ---
    params.bias.tau_gyro = 50;  
    params.bias.tau_acc  = 50;  
    params.bias.tau_mag  = 1;   
    params.bias.tau_baro = 1;

    params.Qnoise.s_dbgx = 0.01;
    params.Qnoise.s_dbgy = 0.01;
    params.Qnoise.s_dbgz = 0.01;
    params.Qnoise.s_dbax = 0.01;
    params.Qnoise.s_dbay = 0.01;
    params.Qnoise.s_dbaz = 0.01;
    params.Qnoise.s_dbmx = 0.01;
    params.Qnoise.s_dbmy = 0.01;
    params.Qnoise.s_dbmz = 0.01;
    params.Qnoise.s_dbbaro = 0.01;

    %% Fin Parameters
    
    params.fin.chord = 0.1; % fin chord [m]
    params.fin.span = 0.05; % fin span [m]
    params.fin.num = 4; % fin num
    params.fin.max_angle = deg2rad(10); % max angle [rad]
    params.fin.root_chord = 0.2;    % From image data [m]
    params.fin.tip_chord = 0.06;    % From image data [m]
    params.fin.span_length = 0.13;  % From image data [m]
    params.fin.sweep = 47;          % From image data [deg]

    %% Calculate Aerodynamic Coefficients
    % Define rocket geometry for coefficient calculation
    rocket.length = params.vehicle.L;
    rocket.diameter = params.vehicle.D_ref * 2;
    rocket.nose_length = 0.3;  % From provided nose cone data
    rocket.nose_type = "haack";
    rocket.cg_position = params.vehicle.L/2;  % Approximate CG position

    % Fin parameters
    rocket.fin_count = params.fin.num;
    rocket.fin_root_chord = params.fin.root_chord;
    rocket.fin_tip_chord = params.fin.tip_chord;
    rocket.fin_span = params.fin.span_length;
    rocket.fin_sweep = params.fin.sweep;

    % Calculate coefficients using Barrowman equations
    aero_coef = calculateRocketCoefficients(rocket);

    % Wind Parameters
    params.wind.v_avg = 10;           % Average wind speed [m/s]
    params.wind.turb_intensity = 0; % Turbulence intensity (0.1-0.2 typical)
    params.wind.direction = 0.01;        % Wind direction [rad]

    % Atmosphere Parameters (for ISA model reference)
    params.environment.T0 = 288.15;   % Sea level temperature [K]
    params.environment.P0 = 101325;   % Sea level pressure [Pa]
    params.environment.a0 = 340.294;  % Sea level speed of sound [m/s]
    params.environment.ro0 = 1.225;   % Sea level density [kg/m^3]

    % Flight Regime Parameters
    params.aero.mach_crit = 0.8;     % Critical Mach number
    params.aero.mach_super = 1.2;    % Supersonic Mach number
    params.aero.aoa_crit = deg2rad(17); % Critical angle of attack [rad]

    % Store calculated coefficients
    params.aero.C_A = aero_coef.C_A;
    params.aero.C_S_beta = aero_coef.C_N_alpha;
    params.aero.C_N_alpha = aero_coef.C_N_alpha;
    params.aero.C_l_p = aero_coef.C_l_p;
    params.aero.C_m_q = aero_coef.C_m_q;
    params.aero.C_m_alpha = aero_coef.C_m_alpha;
    params.aero.C_n_r = aero_coef.C_n_r;
    params.aero.C_n_beta = aero_coef.C_n_beta;
end

function coef = calculateRocketCoefficients(rocket)
    % Calculate rocket aerodynamic coefficients based on Barrowman equations
    
    % Reference dimensions
    S_ref = pi * (rocket.diameter/2)^2;
    
    % Calculate Normal Force Coefficient Derivative (CNα)
    % 1. Nose cone contribution
    if rocket.nose_type == "conical"
        CNa_nose = 2;
    else % haack or ogive
        CNa_nose = 2.4;
    end
    
    % 2. Body contribution
    CNa_body = 2 * (rocket.length - rocket.nose_length) * rocket.diameter / S_ref;
    
    % 3. Fin contribution
    Af = (rocket.fin_root_chord + rocket.fin_tip_chord) * rocket.fin_span / 2;
    AR = 2 * rocket.fin_span^2 / Af;
    
    % Interference factor
    Kfb = 1 + rocket.diameter/(2 * rocket.fin_span);
    
    CNa_fins = Kfb * (4 * rocket.fin_count * (AR / (2 + sqrt(AR^2 + 4)))) * (Af/S_ref);
    
    % Total normal force coefficient
    CNa_total = CNa_nose + CNa_body + CNa_fins;
    
    % Calculate Center of Pressure
    Xcp_nose = 0.466 * rocket.nose_length;
    Xcp_body = rocket.nose_length + (rocket.length - rocket.nose_length)/2;
    MAC = (rocket.fin_root_chord + rocket.fin_tip_chord)/2;
    Xcp_fin = rocket.length - MAC/4;
    
    % Total CP location (weighted average)
    Xcp = (CNa_nose * Xcp_nose + CNa_body * Xcp_body + CNa_fins * Xcp_fin) / CNa_total;
    
    % Calculate coefficients
    coef.C_N_alpha = CNa_total;
    coef.C_A = 0.1 + 0.1 * (rocket.length/rocket.diameter);  % Simplified subsonic estimate
    
    % Roll damping coefficient
    coef.C_l_p = -rocket.fin_count * CNa_fins * rocket.fin_span^2 / (4 * S_ref * rocket.length);
    
    % Pitch and yaw coefficients
    coef.C_m_q = -2 * (Xcp - rocket.cg_position) * CNa_total / rocket.length;
    coef.C_n_r = coef.C_m_q;  % For symmetric rocket
    
    coef.C_m_alpha = -(Xcp - rocket.cg_position) * CNa_total / rocket.length;
    coef.C_n_beta = coef.C_m_alpha;  % For symmetric rocket
    
    % Additional stability parameters
    coef.Xcp = Xcp;
    coef.stability_margin = (Xcp - rocket.cg_position) / rocket.diameter;
end