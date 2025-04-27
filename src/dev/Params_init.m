%% Rocket Parmeter
   
function params = Params_init()
    %% PID Paremeters  
    
    % Angle_PPID gain = [Roll, pitch, Yaw]
 
    params.control.Ka_pp = [ 1, 1, 1 ]'; %Attitude P
   
    params.control.Ka_p = [ 1, 1, 1 ]'; % Rate P   
    params.control.Ka_i = [ 0, 0, 0 ]'; % Rate I    
    params.control.Ka_d = [ 0, 0, 0 ]'; % Rate D  

     
    %% Rocket Parameters
  
    params.vehicle.J_X = 0.061; % [kgm^2]
    params.vehicle.J_Y = 13.4; % [kgm^2]
    params.vehicle.J_Z = 13.4; % [kgm^2] 

    params.vehicle.S_W_ref = 1.1068; % [m^2] % surface area
    params.vehicle.S_A_ref = 0.0132; % [m^2] % diameter area
    
    params.vehicle.m_D = 19.995; % [kg] (Dry mass)
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

    %% Motor Parameters ()

    % Read CSV file
    opts = detectImportOptions('AeroTech_M2400T.csv', 'VariableNamingRule', 'preserve');
    data_stage1 = readtable('AeroTech_M2400T.csv', opts); % Rocket Thrust csv
    
    % Engine dry mass (마지막 질량값이 건조 질량)
    params.motor.E1M_d = data_stage1.("Mass(g)")(end)/1000; % g to kg
    
    % Total Fuel mass
    % Stage 1 - Mass 컬럼 사용
    fuel_mass_stage1 = data_stage1.("Mass(g)")/1000; % g to kg
    
    % interp_timer set - 1000Hz
    time_stage1 = data_stage1.("Time(s)");
    total_time = max(time_stage1) - min(time_stage1);
    sampling_rate = 1000; % Hz setting
    interp_point_1 = round(total_time * sampling_rate) + 1; % +1 -> start point
    new_time_stage1 = linspace(min(time_stage1), max(time_stage1), interp_point_1);
    
    % interpolation
    interp_mass_stage1 = interp1(time_stage1, fuel_mass_stage1, new_time_stage1, 'linear');
    interp_thrust_stage1 = interp1(time_stage1, data_stage1.("Thrust(N)"), new_time_stage1, 'linear');
    params.motor.thrust_func_stage1 = interp_thrust_stage1;
    params.motor.mass_func_stage1 = interp_mass_stage1;
    
    % Engine wet mass (First mass = wet mass)
    params.vehicle.m_W = params.motor.mass_func_stage1(1);
    params.motor.Ti_1 = interp_point_1;

    %% Environment Parameters ()

    params.environment.g = [0, 0, 9.81]'; % [m/s^2]
    params.environment.ro = 1.666; % [kg/m^3]
    params.environment.ro_2 = 1.24; % [kg/m^3]

    %% Aerodynamic Coefficients

    params.aero.C_A = 0.41; % Axial Force Coeff.
    params.aero.C_S_beta = 10.81; % side_alpha Force Coeff.
    params.aero.C_N_alpha = 10.81; % Normal_beta Force Coeff.


    params.aero.C_l_p = -0.00036; % Roll_p Moment Coeff.

    params.aero.C_m_q = -0.0324; % Pitch_q Moment Coeff.
    params.aero.C_m_alpha = -3.2973; % Pitch_alpha Moment Coeff.

    params.aero.C_n_r = -0.0324; % Normal_r Moment Coeff.
    params.aero.C_n_beta = -3.2973; % Normal_beta Moment Coeff.

    %% Fin Parameters

    params.fin.chord = 0.1;  % fin chord
    params.fin.span = 0.05;  % fin span
    params.fin.num = 4;      % fin num
    params.fin.max_angle = deg2rad(10);  % max angle

end  