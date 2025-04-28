clc
clear all
close all

params = Params_init();

X_eq = [0 0 0 0 0 0 0 0 0 0 0 0 0]';
U_eq = [0 0 0 0 0 0 0 0]';


[ A, B, C, D ] = ABSat_dynamcis(X_eq, U_eq, params);

%%
function [ A, B, C, D ] = ABSat_dynamcis(X_eq, U_eq, params)


    % Symbolic variables
    syms Xs Ys Zs Us Vs Ws qw_s qx_s qy_s qz_s p_s q_s R_s real
    syms Us1 Us2 Us3 Us4 Us5 Us6 Us7 Us8 real
    syms M Ts Ds J1 J2 J3 real
    
    %Parameters set

    I = params.Airbearing.I_Z ;
    L = params.Airbearing.D_ref ;
    m = params.Airbearing.m;
    T = params.Airbearing.Thrust;

    Xss = [ Xs; Ys; Zs; Us; Vs; Ws; qw_s; qx_s; qy_s; qz_s; p_s; q_s; R_s ];
    Uss = [ Us1; Us2; Us3; Us4; Us5; Us6; Us7; Us8 ];
    PRs_SET = [M Ts Ds J1 J2 J3]';

    dt = 0.825;

    PR_SET = [m, T, L, 1, 1, I]';

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

    Control_Logic = Ts .* [   0,   -1,    -1,    0,     0,    1,     1,    0;   % X
                             -1,    0,     0,    1,     1,    0,     0,   -1;   % Y
                              0,    0,     0,    0,     0,    0,     0,    0;   % Z
                              0,    0,     0,    0,     0,    0,     0,    0;   % L
                              0,    0,     0,    0,     0,    0,     0,    0;   % M
                            -Ds,   Ds,   -Ds,   Ds,   -Ds,   Ds,   -Ds,   Ds ]; % N

    Fota  = Control_Logic * Uss;

    Fx = Fota( 1 ) ;
    Fy = Fota( 2 ) ;
    Fz = Fota( 3 ) ;

    disp('Matrix Fx:');
    disp(Fx);
    disp('Matrix Fy:');
    disp(Fy);
    disp('Matrix Fz:');
    disp(Fz);

    L_tau = Fota( 4 );
    M_tau = Fota( 5 );
    N_tau = Fota( 6 );

    disp('Matrix L:');
    disp(L_tau);
    disp('Matrix M:');
    disp(M_tau);
    disp('Matrix N:');
    disp(N_tau);


    % Force & Torque
    F_T = [ Fx; Fy; Fz ];
    M_T = [ L_tau; M_tau; N_tau ];

    xs1_dot = RB2I * [ Xss(4); Xss(5); Xss(6) ];
    xs2_dot = F_T / M - cross( [ Xss(10); Xss(11); Xss(12) ], [ Xss(4); Xss(5); Xss(6) ] );
    xs3_dot = dot_q;
    xs4_dot = inv( J ) * ( M_T - cross( [ Xss(10); Xss(11); Xss(12) ], J * [ Xss(10); Xss(11); Xss(12) ] ) );
    
    xsdot = [ xs1_dot; xs2_dot; xs3_dot; xs4_dot ];

    A = jacobian(xsdot, Xss);
    B = jacobian(xsdot, Uss);

    disp('Matrix A:');
    disp(A);

    disp('Matrix B:');
    disp(B);

 
    
    A = double( subs( A, [ Xss; Uss; PRs_SET ], [ X_eq; U_eq; PR_SET ] ) );
    B = double( subs( B, [ Xss; Uss; PRs_SET ], [ X_eq; U_eq; PR_SET ] ) );

    disp('Matrix A:');
    disp(A);

    disp('Matrix B:');
    disp(B);

    [A,B] = c2dm(A,B,[],[],dt)

    disp('Matrix Ad:');
    disp(A);

    disp('Matrix Bd:');
    disp(B);

    C = 0;
    D = 0;

    % C = diag( [ 1, 1, 1, 1 ] );
    % D = zeros(4,8);

end

%%
function params = Params_init()
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
    % Limit Paramters

    params.Airbearing.C1 = 0.2; % [mm]
      
    params.Airbearing.SL1 = 40; % [mm/s]
    params.Airbearing.SL2 = 50; % [mm/s]
    params.Airbearing.SL3 = 70; % [mm/s]
 
    params.Airbearing.C2 = deg2rad(5); % [deg]

    params.Airbearing.SA1 = deg2rad(2);% [deg/s]
    params.Airbearing.SA2 = deg2rad(100); % [deg/s]
    params.Airbearing.SA3 = deg2rad(45); % [deg/s]
  
    % PID Paremeters
 
    % Position_P gain = [X, Y]

    params.Airbearing.Kp_pp = 5; % Velocity P

    % Velocity_PID = [U V]

    params.Airbearing.Kp_p = 15; % Velocity P
    params.Airbearing.Kp_i = 0; % Velocity I
    params.Airbearing.Kp_d = 5; % Velocity D
 
    % Velocity_PID = [U V]


    % Attitude_PID = [Yaw]

    params.Airbearing.Ka_pp = 10;

    params.Airbearing.Ka_p = 8.6; % Attitude P
    params.Airbearing.Ka_i = 0; % Attitude I
    params.Airbearing.Ka_d = 0.3; % Attitude D


    % params.Airbearing.Ka_p = 39; % Attitude P
    % params.Airbearing.Ka_i = 0; % Attitude I
    % params.Airbearing.Ka_d = 38; % Attitude D

    params.ReactionWheel.Kr_p = 4.9;
    params.ReactionWheel.Kr_i = 0.1;
    params.ReactionWheel.Kr_d = 3.8;


    % Reaction Wheel Parameters

    M_motor = 0.27; % [kg] Motor mass
    R_motor = 42; % [mm] Motor radius
    M_disk = 0.97; % [kg] Fly wheel mass
    R_disk = 47; % [mm] Fly wheel radius

    I_motor = 0.5 * M_motor * R_motor^2;
    I_disk = 0.5 * M_disk * R_disk^2;
    
    kv_rating = 100; % KV rating of the motor
    battery_voltage = 22.2; % Voltage of 6-cell LiPo battery in volts

    max_rpm = kv_rating * battery_voltage; % Maximum RPM

    params.Reaction_wheel.omega_wheel_init = 0; % [rad/s] 
    params.Reaction_wheel.omega_limit = max_rpm * 2 * pi / 60; % Convert RPM to rad/s
    params.Reaction_wheel.I_wheel = I_motor + I_disk; % [kg*mm^2] Reaction wheel moment

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     
    % Airbearing Parameters

    params.Airbearing.I_Z =  340; % [kgmm^2]
    params.Airbearing.D_ref = 100; % [mm] (CG to Thruster) 
    params.Airbearing.m = 18; % [kg] (Airbearing mass)
    params.Airbearing.Thrust = 0.07; % [N] (Mean Thrust Force)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
    % EKF Paramters

    params.EKF.P = eye(6);

    params.EKF.H = eye(6);

    params.EKF.Q = 1000 * eye(6);

    params.EKF.R = 0.01 * eye(6);

end


function RI2B = GetDCM_Euler( psi, the, phi )
  
    Cpsi = cos ( psi );
    Spsi = sin ( psi );
    Cthe = cos ( the );
    Sthe = sin ( the );
    Cphi = cos ( phi );
    Sphi = sin ( phi );

    RI2B(1,1) =  Cthe * Cpsi ;
    RI2B(1,2) =  Cthe * Spsi ;
    RI2B(1,3) = -Sthe ;
    RI2B(2,1) = -Cphi * Spsi + Sphi * Sthe * Cpsi ;
    RI2B(2,2) =  Cphi * Cpsi + Sphi * Sthe * Spsi ;
    RI2B(2,3) =  Sphi * Cthe ;
    RI2B(3,1) =  Sphi * Spsi + Cphi * Sthe * Cpsi ;
    RI2B(3,2) = -Sphi * Cpsi + Cphi * Sthe * Spsi ;
    RI2B(3,3) =  Cphi * Cthe ;

end