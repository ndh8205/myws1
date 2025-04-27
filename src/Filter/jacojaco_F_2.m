%% 1) Define symbolic variables
% -- State
syms px py pz real
syms u v w real
syms qw qx qy qz real
syms wx wy wz real

% -- Force parameters
syms Fa_x Fa_y Fa_z real   % Aero etc.
syms Ft_x Ft_y Ft_z real   % Thrust
syms Fr_x Fr_y Fr_z real   % RCS
syms Fn_x Fn_y Fn_z real   % Noise
% => If you have expansions for these in terms of alpha etc., insert here

% -- Gravity: we put it "directly" => Fgrav = R_I2B(q)*[0;0;m*g]
syms m g real  % mass, gravity

% -- Moments
syms Ma_x Ma_y Ma_z real
syms Mr_x Mr_y Mr_z real
syms Mn_x Mn_y Mn_z real
syms Cpx real         % cross(Cp2CG, Faero)
%   cross([Cpx,0,0],[Fa_x,Fa_y,Fa_z]) => [0, -Cpx*Fa_z, Cpx*Fa_y]

% -- Inertia
syms Jx Jy Jz real

% -- dt tau
syms dt tau real

% -- sigma _ Qk
syms s_x s_y s_z s_u s_v s_w s_qw s_qx s_qy s_qz s_wx s_wy s_wz

%% 2) Build state vector X
X_sym = [
  px; py; pz;    % 1..3
  u;  v;  w;     % 4..6
  qw; qx; qy; qz;% 7..10
  wx; wy; wz     % 11..13
];

%% 3) Continuous-time dynamics

% 3.1) p_dot = R_B2I(q)* v_b
v_b = [u; v; w];
R_B2I = quat_to_DCM(qw,qx,qy,qz); % body->inertial
p_dot = R_B2I * v_b;  % (3x1)

% 3.2) total forces in body
Faero = [Fa_x; Fa_y; Fa_z];
Fth   = [Ft_x; Ft_y; Ft_z];
Frc   = [Fr_x; Fr_y; Fr_z];
Fnoi  = [Fn_x; Fn_y; Fn_z];

% => gravity in inertial: g_inertial = [0;0; +g]
% => transform to body: Fgrav_b = R_I2B(q)* (m*[0;0;g])
%    주의: R_I2B = R_B2I^T, 아래서 transpose 써도 됨.
R_I2B = (R_B2I).';  % invert
g_inertial = [0; 0; g];
Fgrav_b = R_I2B * (m * g_inertial);

F_total_body = Faero + Fgrav_b + Fth + Frc + Fnoi;

% v_b_dot = (1/m)*F_total_body - cross( [wx,wy,wz], [u,v,w] )
omega_b = [wx; wy; wz];
v_b_dot = (1/m)*F_total_body - cross(omega_b, v_b);

% 3.3) q_dot = 0.5 * Omega(omega)* q
q_vec = [qw; qx; qy; qz];
Omega = [
   0,   -wx,  -wy,  -wz;
   wx,    0,   wz,  -wy;
   wy,  -wz,    0,   wx;
   wz,   wy,  -wx,    0
];
q_dot = 0.5 * Omega * q_vec;

% 3.4) w_dot = J^-1( M_total - cross(omega, J*omega) )
%    M_total = Maero + cross([Cpx;0;0], Faero) + Mr + Mn
Maer = [Ma_x; Ma_y; Ma_z];
Mrcs = [Mr_x; Mr_y; Mr_z];
Mnoi = [Mn_x; Mn_y; Mn_z];

crossCpFa = [0; -Cpx*Fa_z; Cpx*Fa_y];
M_total = Maer + crossCpFa + Mrcs + Mnoi;

Jmat = diag([Jx, Jy, Jz]);
cross_iner = cross(omega_b, Jmat*omega_b);
w_dot = Jmat \ ( M_total - cross_iner );

%% 4) f_cont = [p_dot; v_b_dot; q_dot; w_dot]
f_cont = [
   p_dot;   % 3
   v_b_dot; % 3
   q_dot;   % 4
   w_dot    % 3
];  % => 13x1

%% 5) Jacobian A_sym = df_cont/dX (13x13)
A_sym = jacobian(f_cont, X_sym);

%% 6) Discretize => F_sym = I + A_sym*dt
nX = length(X_sym);
I_n = eye(nX);
F_sym = I_n + A_sym*dt;

%% 7) simplify (optional)
A_sym = simplify(A_sym);
F_sym = simplify(F_sym);

%% 8) F matrix Output
disp('===========================================================');
disp('State vector X_sym = ');
disp(X_sym);

disp('===========================================================');
disp('Continuous-time f_cont(X) = ');
disp(f_cont);

disp('===========================================================');
disp('Jacobian A_sym = d(f_cont)/dX (13x13) = ');
disp(A_sym);

disp('===========================================================');
disp('Discrete-time F_sym = I + A_sym*dt = ');
disp(F_sym);



%% 8) Q matrix calc

Qs = diag( [ s_x; s_y; s_z; s_u; s_v; s_w; s_qw; s_qx; s_qy; s_qz; s_wx; s_wy; s_wz ] );
Ws = diag( [ 0; 0; 0; 1/m; 1/m; 1/m; 0; 0; 0; 0; 1/Jx; 1/Jy; 1/Jz ] );
F_sym_tr = transpose(F_sym);
Qm = F_sym * Ws * Qs * Ws * F_sym_tr;

Qk = int( Qm, tau, 0, dt );
Qk = simplify(Qk);
disp('===========================================================');
disp('Qk');
disp(Qk);


%% quaternion->DCM (body->inertial)
function R = quat_to_DCM(qw,qx,qy,qz)
    % we can assume it's normalized, or do it here if needed
    R = sym(zeros(3,3));
    R(1,1) = qw^2 + qx^2 - qy^2 - qz^2;
    R(1,2) = 2*(qx*qy - qw*qz);
    R(1,3) = 2*(qx*qz + qw*qy);
    
    R(2,1) = 2*(qx*qy + qw*qz);
    R(2,2) = qw^2 - qx^2 + qy^2 - qz^2;
    R(2,3) = 2*(qy*qz - qw*qx);
    
    R(3,1) = 2*(qx*qz - qw*qy);
    R(3,2) = 2*(qy*qz + qw*qx);
    R(3,3) = qw^2 - qx^2 - qy^2 + qz^2;
end

% matlabFunction( Qk, ...
%     'File', 'calcQk_auto.m', ...  % 생성될 M파일 이름
%     'Vars', { ...
%        % -- state variables (that actually appear in A_sym) --
%        u, v, w, ...
%        qw, qx, qy, qz, ...
%        wx, wy, wz, ...
%        m, g, Jx, Jy, Jz, dt, ...
%        s_x, s_y, s_z, ...
%        s_u, s_v, s_w, ...
%        s_qw, s_qx, s_qy, s_qz, ...
%        s_wx, s_wy, s_wz ...
%     } );

% disp('Done!  "calcQk_explicit_minimal.m" has been generated.');