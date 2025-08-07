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
    params.vehicle.RCS_T = 3; % [N] % Reaction control system thruster

    % Store inertia tensor directly as a matrix
    params.vehicle.J = [params.vehicle.J_X, 0, 0;
                       0, params.vehicle.J_Y, 0;
                       0, 0, params.vehicle.J_Z];

    %% Motor Parameters - 추진력과 질량변화
    % Read CSV file
    opts = detectImportOptions('AeroTech_M2400T.csv', 'VariableNamingRule', 'preserve');
    data_stage1 = readtable('AeroTech_M2400T.csv', opts);

    % Engine dry mass
    params.motor.E1M_d = data_stage1.("Mass(g)")(end)/1000; % g to kg

    % Interpolation setup
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

    %% Controller Parameters
    % Angle_PPID gain = [Roll, pitch, Yaw]
    params.control.Ka_pp = [0, 0, 0]'; %Attitude P
    params.control.Ka_p = [0, 0, 0]'; % Rate P
    params.control.Ka_i = [0, 0, 0]'; % Rate I
    params.control.Ka_d = [0, 0, 0]'; % Rate D