%==========================================================================
% Example: Symbolic derivation of F matrix for 6DOF rocket equations
% with a 13-state vector:
%   X = [ px, py, pz,  u,  v,  w,  qw, qx, qy, qz,  wx, wy, wz ]
%
% We consider the system (continuous):
%
%   (1)  p_dot = R_B2I(q) * V_b         (V_b = [u;v;w])
%   (2)  V_b_dot = (1/m)*[Fx;Fy;Fz]  -  w_body x V_b
%   (3)  q_dot  = 0.5 * Omega(w_body)*q
%   (4)  w_dot  = J^{-1} ( [Mx;My;Mz] - w x Jw )
%
% * Fx,Fy,Fz, Mx,My,Mz, m, Jx,Jy,Jz 등은 추가 심볼릭 변수로 둠.
% * For actual rocket, you'll put gravity, aero forces, RCS, etc. into Fx,Fy,Fz...
%
% We'll do:
%   f_cont(X) = [p_dot; V_b_dot; q_dot; w_dot]
%   A_sym = df_cont/dX
%   F_sym = I + A_sym * dt   (simple Euler discretization)
%
% (C) Example by ChatGPT
%==========================================================================

%-----------------------------
% 1) Define symbolic variables
%-----------------------------
syms px py pz real         % position in inertial frame
syms u v w real            % velocity in body frame
syms qw qx qy qz real      % quaternion (inertial->body or body->inertial)
syms wx wy wz real         % angular velocity (body frame)
syms Fx Fy Fz real         % external forces (body frame)
syms Mx My Mz real         % external torques (body frame)
syms m real                % mass
syms Jx Jy Jz real         % moments of inertia (diagonal assumed)
syms dt real               % time step for discrete F

% Construct state vector X
X_sym = [ px; py; pz;  u; v; w;  qw; qx; qy; qz;  wx; wy; wz ];

%-----------------------------
% 2) Define rotation from quaternion
%    R_B2I(q) : body->inertial
%    We want p_dot = R_B2I * [u;v;w].
%-----------------------------
% Symbolic DCM from quaternion (assuming q=[qw,qx,qy,qz], normalized)
R11 = qw^2 + qx^2 - qy^2 - qz^2;
R12 = 2*(qx*qy - qw*qz);
R13 = 2*(qx*qz + qw*qy);

R21 = 2*(qx*qy + qw*qz);
R22 = qw^2 - qx^2 + qy^2 - qz^2;
R23 = 2*(qy*qz - qw*qx);

R31 = 2*(qx*qz - qw*qy);
R32 = 2*(qy*qz + qw*qx);
R33 = qw^2 - qx^2 - qy^2 + qz^2;

R_B2I = [R11, R12, R13;
         R21, R22, R23;
         R31, R32, R33];

v_b = [u; v; w];

%-----------------------------
% 3) Continuous-time dynamics
%-----------------------------
% 3a) p_dot = R_B2I * v_b
p_dot = R_B2I * v_b;  % 3x1

% 3b) V_b_dot
%     Suppose V_b_dot = (1/m)*[Fx; Fy; Fz] - w_body x v_b
w_body = [wx; wy; wz];
force_b = [Fx; Fy; Fz];
v_b_dot = (1/m)*force_b - cross(w_body, v_b);

% 3c) q_dot = 0.5 * Omega(w_body)*q
%     Omega(w) = [0, -wx, -wy, -wz;
%                 wx,   0,  wz, -wy;
%                 wy, -wz,   0,  wx;
%                 wz,  wy, -wx,   0]
Omega = [ 0,    -wx,  -wy,  -wz;
          wx,    0 ,   wz,  -wy;
          wy,   -wz,   0 ,   wx;
          wz,    wy,  -wx,   0 ];

q_vec = [qw; qx; qy; qz];
q_dot = 0.5 * Omega * q_vec;  % 4x1

% 3d) w_dot = J^{-1} * ( [Mx,My,Mz]^T - w x (J w) )
%     J = diag(Jx, Jy, Jz)
Jmat = diag([Jx, Jy, Jz]);
M_b = [Mx; My; Mz];
cross_term = cross(w_body, Jmat*w_body);
w_dot = inv(Jmat) * ( M_b - cross_term );  % 3x1

% Concatenate into f_cont
f_cont = [p_dot; v_b_dot; q_dot; w_dot];  % 13x1

%---------------------------------------
% 4) Jacobian A_sym = df_cont/dX (13x13)
%---------------------------------------
A_sym = jacobian(f_cont, X_sym);

%---------------------------------------
% 5) Discretize: F_sym ~ I + A_sym * dt
%    (simple Euler approximation)
%---------------------------------------
nX = length(X_sym);
I_n = eye(nX);
F_sym = I_n + A_sym * dt;

% Optional: simplify
A_sym = simplify(A_sym);
F_sym = simplify(F_sym);

%---------------------------------------
% Output or display
%---------------------------------------
disp('======================================================');
disp('Symbolic state vector X_sym = ');
disp(X_sym);

disp('======================================================');
disp('Symbolic continuous-time dynamics f_cont(X) = ');
disp(f_cont);

disp('======================================================');
disp('Symbolic A_sym = d f_cont / dX = ');
disp(A_sym);

disp('======================================================');
disp('Discrete-time F_sym ~ I + A_sym*dt = ');
disp(F_sym);