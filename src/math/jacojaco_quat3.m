clc
clear all
close all

% Symbolic variables declaration
syms Xs Ys Zs Us Vs Ws qw_s qx_s qy_s qz_s p_s q_s r_s real
syms Us1 Us2 Us3 real
syms m F_T ga1 ga2 x_Nozzle g ro S_ref D_ref x_ref real
syms C_A C_S_beta C_N_alpha C_l_p C_m_q C_m_alpha C_n_r C_n_beta real
syms I11 I22 I33 dt real

params = Params_init();

X_eq = [Xs; Ys; Zs; Us; Vs; Ws; qw_s; qx_s; qy_s; qz_s; p_s; q_s; r_s];
U_eq = [Us1; Us2; Us3];

[A_sym, B_sym, A_discrete, B_discrete] = TVC_rocket_dynamics(X_eq, U_eq, params);

function [A_sym, B_sym, A_discrete, B_discrete] = TVC_rocket_dynamics(X_eq, U_eq, params)
    % Unpack state and input vectors
    Xs = X_eq(1); Ys = X_eq(2); Zs = X_eq(3);
    Us = X_eq(4); Vs = X_eq(5); Ws = X_eq(6);
    qw_s = X_eq(7); qx_s = X_eq(8); qy_s = X_eq(9); qz_s = X_eq(10);
    p_s = X_eq(11); q_s = X_eq(12); r_s = X_eq(13);
    
    Us1 = U_eq(1); Us2 = U_eq(2); Us3 = U_eq(3);
    
    % Declare symbolic variables for parameters
    syms m F_T ga1 ga2 x_Nozzle g ro S_ref D_ref x_ref real
    syms C_A C_S_beta C_N_alpha C_l_p C_m_q C_m_alpha C_n_r C_n_beta real
    syms I11 I22 I33 dt real
    
    % Quaternion to DCM
    R_B2I_quat = [
        qw_s^2 + qx_s^2 - qy_s^2 - qz_s^2, 2*(qx_s*qy_s - qw_s*qz_s), 2*(qw_s*qy_s + qx_s*qz_s);
        2*(qx_s*qy_s + qw_s*qz_s), qw_s^2 - qx_s^2 + qy_s^2 - qz_s^2, 2*(qy_s*qz_s - qw_s*qx_s);
        2*(qx_s*qz_s - qw_s*qy_s), 2*(qw_s*qx_s + qy_s*qz_s), qw_s^2 - qx_s^2 - qy_s^2 + qz_s^2
    ];
    
    RB2I = R_B2I_quat;
    RI2B = RB2I.';

    % Quaternion derivative
    qbar = [
        qw_s, -qx_s, -qy_s, -qz_s;
        qx_s,  qw_s, -r_s,   q_s;
        qy_s,  r_s,   qw_s, -p_s;
        qz_s, -q_s,   p_s,   qw_s
    ];
    O_bq = [0; p_s; q_s; r_s];
    dot_q = simplify(0.5 * qbar * O_bq);

    % V_B angle Define
    V_B = [Us; Vs; Ws];
    V_B_norm = sqrt(Us^2 + Vs^2 + Ws^2);
    alpha = atan2(Ws, Us);
    beta = asin(Vs / V_B_norm);

    % force & Moment
    q_aero = 0.5 * ro * V_B_norm^2; % dynamic pressure
    Damp_M = D_ref / 2 * V_B_norm;
      
    C_A = C_A * q_aero * S_ref; % Axial Force
    C_S = C_S_beta * beta * q_aero * S_ref; % Side Force
    C_N = C_N_alpha * alpha * q_aero * S_ref; % Normal Force

    C_l = (Damp_M * C_l_p * p_s) * q_aero * S_ref * D_ref; % Aero Roll Moment
    C_m = (Damp_M * C_m_q * q_s + C_m_alpha * alpha) * q_aero * S_ref * D_ref; % Aero Pitch Moment
    C_n = (Damp_M * C_n_r * r_s + C_n_beta * beta) * q_aero * S_ref * D_ref; % Aero Yaw Moment

    Cp2CG = [x_ref; 0; 0];
    F_A = [-C_A; -C_S; -C_N]; % Force Vector (Aero) - body frame
    F_G = RI2B * [0; 0; m*g]; % Force Vector (Gravitation) - body frame
    F_T = [F_T * cos(ga1) * cos(ga2); F_T * cos(ga1) * sin(ga2); -F_T * sin(ga1)]; % Thrust Force Vector - body frame
    M_A_Mrp = [C_l; C_m; C_n]; % Moment Vector (Aero) - MRP frame
    M_A_body = M_A_Mrp + cross(F_A, Cp2CG); % Moment Vector (Aero) - body frame
    M_T = [0; -F_T * sin(ga1) * abs(x_Nozzle); -F_T * cos(ga1) * sin(ga2) * abs(x_Nozzle)]; % Thrust Moment Vector - body frame 
    M_RCS = [Us1; 0; 0];

    % Inertia matrix
    I = diag([I11, I22, I33]);
    omega = [p_s; q_s; r_s];

    % State derivatives
    x1_dot = RB2I * V_B; % inertial frame
    x2_dot = ((F_A + F_G + F_T) / m) - cross(omega, V_B); % body frame
    x3_dot = dot_q; % inertial frame
    x4_dot = inv(I) * ((M_A_body + M_T + M_RCS) - cross(omega, I * omega)); % body frame

    xsdot = [x1_dot; x2_dot; x3_dot; x4_dot];

    % Jacobians
    A_sym = jacobian(xsdot, X_eq);
    B_sym = jacobian(xsdot, U_eq);

    % Display symbolic matrices
    disp('Symbolic A matrix:');
    disp(A_sym);
    disp('Symbolic B matrix:');
    disp(B_sym);

    % Discretization
    A_discrete = eye(size(A_sym)) + dt * A_sym;
    B_discrete = dt * B_sym;

    % Display discrete matrices
    disp('Discrete A matrix:');
    disp(A_discrete);
    disp('Discrete B matrix:');
    disp(B_discrete);
end

function params = Params_init()
    % This function should initialize all the parameters used in the TVC_rocket_dynamics function
    % The actual values are not shown here as they were not provided in the original code
    params.rocket = struct('S_ref', [], 'D_ref', [], 'I', [], 'm_f', [], 'x_Nozzle', [], 'gimbal_limit', [], 'x_ref', []);
    params.environment = struct('g', [], 'ro', []);
    params.aero = struct('C_A', [], 'C_S_beta', [], 'C_N_alpha', [], 'C_l_p', [], 'C_m_q', [], 'C_m_alpha', [], 'C_n_r', [], 'C_n_beta', []);
    params.motor = struct('thrust_func_stage1', [], 'thrust_func_stage2', [], 'mass_func_stage1', [], 'mass_func_stage2', []);
    params.Simulation = struct('Sim_Loop_Hz', []);
end