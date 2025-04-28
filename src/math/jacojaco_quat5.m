clc
clear all
close all

params = Params_init();

X_eq = [0 0 0 0 0 0 0 0 0 0 0 0 0]';
U_eq = [0 0 0 0 0 0 0 0]';


[ A, B, C, D ] = ABTVC_dynamcis(X_eq, U_eq, params);

function [ A, B, C, D ] = ABTVC_dynamcis(X_eq, U_eq, params)

    % Symbolic variables
    syms Xs Ys Zs Us Vs Ws qw_s qx_s qy_s qz_s p_s q_s R_s real
    syms Us1 Us2 Us3 real
    syms M Ts Ds J1 J2 J3 real
    
    % Parameters set
    I = params.rocket.I;
    L = params.rocket.D_ref;
    m = params.rocket.m_f;
    T = params.rocket.S_ref;

    Xss = [ Xs; Ys; Zs; Us; Vs; Ws; qw_s; qx_s; qy_s; qz_s; p_s; q_s; R_s ];
    Uss = [ Us1; Us2; Us3 ];
    PRs_SET = [M Ts Ds J1 J2 J3]';

    dt = 0.825;

    PR_SET = [m, T, L, diag(I)]';

    J = [ J1, 0, 0; 0, J2, 0; 0, 0, J3 ]; 

    disp('Matrix Intertial Tensor:');
    disp(J);

    R_B2I_quat = sym('R_B2I_quat', [3, 3]);
    R_B2I_quat(1,1) = Xss(7)^2 + Xss(8)^2 - Xss(9)^2 - Xss(10)^2;
    R_B2I_quat(1,2) = 2*(Xss(8)*Xss(9) - Xss(7)*Xss(10));
    R_B2I_quat(1,3) = 2*(Xss(7)*Xss(9) + Xss(8)*Xss(10));
    R_B2I_quat(2,1) = 2*(Xss(8)*Xss(9) + Xss(7)*Xss(10));
    R_B2I_quat(2,2) = Xss(7)^2 - Xss(8)^2 + Xss(9)^2 - Xss(10)^2;
    R_B2I_quat(2,3) = 2*(Xss(9)*Xss(10) - Xss(7)*Xss(8));
    R_B2I_quat(3,1) = 2*(Xss(8)*Xss(10) - Xss(7)*Xss(9));
    R_B2I_quat(3,2) = 2*(Xss(9)*Xss(10) + Xss(7)*Xss(8));
    R_B2I_quat(3,3) = Xss(7)^2 - Xss(8)^2 - Xss(9)^2 + Xss(10)^2;
    
    RB2I = R_B2I_quat;
    RI2B = RB2I.';  % 전치 연산
    
    disp('Matrix RI2B:');
    disp(RI2B);
    
    disp('Matrix RB2I:');
    disp(RB2I);

    qbar = sym('qbar', [4, 4]);

    qbar(1,:) = [ Xss(7),  -Xss(8),  -Xss(9),  -Xss(10) ];
    qbar(2,:) = [ Xss(8),   Xss(7),  -Xss(12),  Xss(11) ];
    qbar(3,:) = [ Xss(9),   Xss(12),   Xss(7), -Xss(11) ];
    qbar(4,:) = [ Xss(10), -Xss(11),  Xss(11),  Xss(7)  ];

    O_bq = [ sym(0); Xss(11); Xss(12); Xss(13) ];

    dot_q = simplify(0.5 * qbar * O_bq);

    disp('Symbolic quaternion derivative:');
    disp(dot_q);

    % Control Logic for TVC
    % Assume Ts is the thrust and Ds is the distance for generating torque
    Control_Logic = [1, 0, 0;  % Thrust in X direction
                     0, 1, 0;  % Thrust in Y direction
                     0, 0, 1]; % Torque generation

    Fota  = Control_Logic * Uss;

    Fx = Fota(1);
    Fy = Fota(2);
    Fz = Fota(3);

    disp('Matrix Fx:');
    disp(Fx);
    disp('Matrix Fy:');
    disp(Fy);
    disp('Matrix Fz:');
    disp(Fz);

    L_tau = Fota(4);
    M_tau = Fota(5);
    N_tau = Fota(6);

    disp('Matrix L:');
    disp(L_tau);
    disp('Matrix M:');
    disp(M_tau);
    disp('Matrix N:');
    disp(N_tau);

    % Force & Torque
    F_T = [Fx; Fy; Fz];
    M_T = [L_tau; M_tau; N_tau];

    xs1_dot = RB2I * [Xss(4); Xss(5); Xss(6)];
    xs2_dot = F_T / M - cross([Xss(10); Xss(11); Xss(12)], [Xss(4); Xss(5); Xss(6)]);
    xs3_dot = dot_q;
    xs4_dot = inv(J) * (M_T - cross([Xss(10); Xss(11); Xss(12)], J * [Xss(10); Xss(11); Xss(12)]));

    xsdot = [xs1_dot; xs2_dot; xs3_dot; xs4_dot];

    A = jacobian(xsdot, Xss);
    B = jacobian(xsdot, Uss);

    disp('Matrix A:');
    disp(A);

    disp('Matrix B:');
    disp(B);

    A = double(subs(A, [Xss; Uss; PRs_SET], [X_eq; U_eq; PR_SET]));
    B = double(subs(B, [Xss; Uss; PRs_SET], [X_eq; U_eq; PR_SET]));

    disp('Matrix A:');
    disp(A);

    disp('Matrix B:');
    disp(B);

    [A, B] = c2dm(A, B, [], [], dt);

    disp('Matrix Ad:');
    disp(A);

    disp('Matrix Bd:');
    disp(B);

    C = 0;
    D = 0;

    % C = diag([1, 1, 1, 1]);
    % D = zeros(4, 8);

end

%% Rocket Parmeter
   
function params = Params_init()


    % Simulation Paramters

    params.Simulation.Sim_Loop_Hz = 1000;
    params.Simulation.time = 30;

              
    %% PID Paremeters  
    
    % Angle_P gain = [Roll, pitch, Yaw]
 
    params.rocket.Ka_pp = [ 2, 13, 1 ]'; %Attitude P
   
 
    params.rocket.Ka_p = [ 2, 1114.29, 285.71  ]'; % Rate P   
    params.rocket.Ka_i = [ 0, 428.57, 0 ]'; % Rate I    
    params.rocket.Ka_d = [ 0.01, 1.428, 0.84 ]'; % Rate D  
     
    %% Rocket Parameters
  
    params.rocket.I_X = 0.027; % [kgm^2]
    params.rocket.I_Y = 1.05; % [kgm^2]
    params.rocket.I_Z = 1.05; % [kgm^2]
    params.rocket.S_ref = 0.009503; % [m^2]
    params.rocket.D_ref = 0.16; % [m]
    params.rocket.x_ref = 0.11; % [m]
    params.rocket.m_f = 4.476; % [kg] (Rocket mass)
    params.rocket.x_Nozzle = -0.6; % [m]
    params.rocket.gimbal_limmit = deg2rad(10) ; % input = [deg] // output = [rad]
    params.rocket.ga = [deg2rad(0), deg2rad(0), deg2rad(0)]'; % input = [deg] // output = [rad] => gimbal move [0 ,pitch, yaw]
 
    % Store inertia tensor directly as a matrix
    params.rocket.I = [params.rocket.I_X, 0, 0;
                       0, params.rocket.I_Y, 0; 
                       0, 0, params.rocket.I_Z];

    %% Motor Parameters

    % Read CSV file
    opts = detectImportOptions('stage1_ver1.csv', 'VariableNamingRule', 'preserve');
    data_stage1 = readtable('stage1_ver1.csv', opts);
    opts = detectImportOptions('stage2_ver1.csv', 'VariableNamingRule', 'preserve');
    data_stage2 = readtable('stage2_ver1.csv', opts);
    
    % Engine dry mass 

    params.motor.E1M_d = 0.476;
    params.motor.E2M_d = 0.440;

    % Total Fuel mass 

    % Stage 1
    fuel_mass_stage1 = ( data_stage1.("Propellant Mass(G1;g)") + data_stage1.("Propellant Mass(G2;g)") + ...
                        data_stage1.("Propellant Mass(G3;g)") + data_stage1.("Propellant Mass(G4;g)") + ...
                        data_stage1.("Propellant Mass(G5;g)") + data_stage1.("Propellant Mass(G6;g)") + ...
                        data_stage1.("Propellant Mass(G7;g)") ) / 1000;
    
    % Stage 2
    fuel_mass_stage2 = ( data_stage2.("Propellant Mass(G1;g)") + data_stage2.("Propellant Mass(G2;g)") + ...
                        data_stage2.("Propellant Mass(G3;g)") + data_stage2.("Propellant Mass(G4;g)") ) / 1000;

    
    % interp_timer set
    time_stage1 = data_stage1.("Time(s)");
    time_stage2 = data_stage2.("Time(s)");

    TI1 = size(time_stage1,1);
    TI2 = size(time_stage2,1);

    interp_point_1 =  TI1 * 10 ;
    interp_point_2 =  TI2 * 10 ;
    
    new_time_stage1 = linspace(min(time_stage1), max(time_stage1), interp_point_1); % 1000개의 점으로 보간
    new_time_stage2 = linspace(min(time_stage2), max(time_stage2), interp_point_2); % 1000개의 점으로 보간

    % interpolation
    interp_mass_stage1 = interp1(time_stage1, fuel_mass_stage1, new_time_stage1, 'linear');
    interp_thrust_stage1 = interp1(time_stage1, data_stage1.("Thrust(N)"), new_time_stage1, 'linear');

    interp_mass_stage2 = interp1(time_stage2, fuel_mass_stage2, new_time_stage2, 'linear');
    interp_thrust_stage2 = interp1(time_stage2, data_stage2.("Thrust(N)"), new_time_stage2, 'linear');


    params.motor.thrust_func_stage1 = interp_thrust_stage1;
    params.motor.mass_func_stage1 = interp_mass_stage1;

    params.motor.thrust_func_stage2 = interp_thrust_stage2;
    params.motor.mass_func_stage2 = interp_mass_stage2;

    % Engine wet mass 

    params.motor.E1M_w = params.motor.E1M_d +  params.motor.mass_func_stage1(1);
    params.motor.E2M_w = params.motor.E2M_d + params.motor.mass_func_stage2(1);

    % Timer set

    params.motor.Ti_1 = interp_point_1;
    params.motor.Ti_2 = interp_point_2;

    params.motor.stage2_ignition_time = 11.7; % 1단 연소 종료 시점에 2단 점화
    params.motor.land_sig = 1;

    %% Environment Parameters

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

    %% Reaction Wheel Parameters

    M_motor = 0.0916; % [kg] Motor mass
    R_motor = 0.22; % [m] Motor radius
    M_disk = 0.200; % [kg] Fly wheel mass
    R_disk = 0.47; % [m] Fly wheel radius

    I_motor = 0.5 * M_motor * R_motor^2;
    I_disk = 0.5 * M_disk * R_disk^2;
    
    kv_rating = 1800; % KV rating of the motor
    battery_voltage = 11.1; % Voltage of 3-cell LiPo battery in volts

    max_rpm = kv_rating * battery_voltage; % Maximum RPM

    params.reaction_wheel.omega_wheel_init = 0; % [rad/s] 
    params.reaction_wheel.omega_limit = max_rpm * 2 * pi / 60; % Convert RPM to rad/s
    params.reaction_wheel.I_wheel = I_motor + I_disk; % [kg*m^2] Reaction wheel moment
     
end 
