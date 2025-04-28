%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% makeFQ_eskf_22.m
%
% "오차 상태 기반 6DoF + Bias 통합" (22차) 심볼릭 전개
%   => F_error(22x22), Q_error(22x22) 산출
%
%  - deltaX_22 = [ dpx dpy dpz  dvx dvy dvz  dthx dthy dthz  dwx dwy dwz
%                  db_gx db_gy db_gz  db_ax db_ay db_az
%                  db_mx db_my db_mz  db_baro ]^T
%  - matlabFunction()을 통해 calcFeskf_22_auto.m, calcQeskf_22_auto.m 생성
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; clc;

%% ========== 1) 심볼릭 변수 정의 ===========
% (A) 공칭 상태(6DoF) - 12차
syms px_hat py_hat pz_hat real
syms u_hat v_hat w_hat real
syms qw_hat qx_hat qy_hat qz_hat real
syms wx_hat wy_hat wz_hat real

% (B) 오차 상태(12차: 위치,속도,자세,각속도)
syms dpx dpy dpz real
syms du dv dw real
syms dthx dthy dthz real
syms dwx dwy dwz real

% (C) 바이어스 오차 상태(10차)
syms dbgx dbgy dbgz real
syms dbax dbay dbaz real
syms dbmx dbmy dbmz real
syms dbbaro real  % 예: 기압계 바이어스

% 합쳐서 22차
deltaX_sym = [
    dpx; dpy; dpz;    % 1~3
    du;  dv;  dw;     % 4~6
    dthx; dthy; dthz; % 7~9
    dwx; dwy; dwz;    % 10~12
    dbgx; dbgy; dbgz; % 13~15
    dbax; dbay; dbaz; % 16~18
    dbmx; dbmy; dbmz; % 19~21
    dbbaro           % 22
];

% (D) 외력/모멘트/파라미터 심볼릭(질문 코드와 동일하게)
syms Fa_x Fa_y Fa_z real
syms Ft_x Ft_y Ft_z real
syms Fr_x Fr_y Fr_z real
syms Fn_x Fn_y Fn_z real
syms m g real
syms Ma_x Ma_y Ma_z real
syms Mr_x Mr_y Mr_z real
syms Mn_x Mn_y Mn_z real
syms Cpx real
syms Jx Jy Jz real
syms dt tau real

% (E) 잡음 세기 (EKF process noise 등)
%  - 여기선 예시로 22개 잡음 모두 정의할 수도 있음
%  - 혹은 필요한 것만 심볼릭 처리
syms s_px s_py s_pz real
syms s_u s_v s_w real
syms s_thx s_thy s_thz real
syms s_wx_ s_wy_ s_wz_ real
% + 바이어스 관련 잡음(10개)
syms s_dbgx s_dbgy s_dbgz real
syms s_dbax s_dbay s_dbaz real
syms s_dbmx s_dbmy s_dbmz real
syms s_dbbaro real

%% ========== 2) 공칭 회전행렬 ===========
R_B2I_hat = quat_to_DCM(qw_hat, qx_hat, qy_hat, qz_hat);
R_I2B_hat = R_B2I_hat.';

%% ========== 3) 실제 상태 vs 공칭 + 오차(12차만) ==========
%   (bias는 센서모델에 따라 다를 수 있으나, 여기서는 '실제=공칭+bias'라기보단
%    'bias오차 db'를 별도 1차 지연 등으로 처리.)
px = px_hat + dpx;  py = py_hat + dpy;  pz = pz_hat + dpz;
u  = u_hat + du;    v  = v_hat + dv;    w  = w_hat + dw;
wx = wx_hat + dwx;  wy = wy_hat + dwy;  wz = wz_hat + dwz;

% 소각벡터 -> delta_quat ~ [1; 0.5*dthx; 0.5*dthy; 0.5*dthz]
syms smallOne real
smallOne = sym(1);
qw_err = smallOne;
qx_err = 0.5*dthx;
qy_err = 0.5*dthy;
qz_err = 0.5*dthz;

q_err = [qw_err; qx_err; qy_err; qz_err];
q_hat = [qw_hat; qx_hat; qy_hat; qz_hat];
q_actual = quatMultiply(q_hat, q_err);

%% ========== 4) 원래 6DoF 동역학(공칭+오차) ==========
v_b = [u; v; w];
p_dot = R_B2I_hat * v_b;

Faero = [Fa_x; Fa_y; Fa_z];
Fth   = [Ft_x; Ft_y; Ft_z];
Frc   = [Fr_x; Fr_y; Fr_z];
Fnoi  = [Fn_x; Fn_y; Fn_z];
Fsum_b = Faero + Fth + Frc + Fnoi;

g_inertial = [0; 0; g];
Fgrav_b    = R_I2B_hat*(m*g_inertial);
F_total_b  = Fsum_b + Fgrav_b;

omega_b    = [wx; wy; wz];
v_dot = (1/m)*F_total_b - cross(omega_b, v_b);

Omega = omegaMat(wx, wy, wz);
q_dot = 0.5 * Omega * q_actual;

Maer = [Ma_x; Ma_y; Ma_z];
Mrcs = [Mr_x; Mr_y; Mr_z];
Mnoi = [Mn_x; Mn_y; Mn_z];
crossCpFa = [0; -Cpx*Fa_z; Cpx*Fa_y];
M_total = Maer + crossCpFa + Mrcs + Mnoi;

Jmat = diag([Jx, Jy, Jz]);
cross_iner = cross(omega_b, Jmat*omega_b);
w_dot = Jmat \ ( M_total - cross_iner );

%% ========== 5) 바이어스 동역학 예시(FOGM 등) ==========
%   예: dot(db) = -1/tau * db + w_b
%   여기서는 tau_gyro, tau_acc 등 심볼릭으로 둘 수도 있음.
%   간단히 tau=const 등 가정으로 placeholder
syms tau_gyro tau_acc tau_mag tau_baro real

% gyro bias: dbgx,dbgy,dbgz
db_gyro_dot = - (1/tau_gyro)*[dbgx; dbgy; dbgz];  % + (잡음) 나중에 반영
% accel bias
db_acc_dot = - (1/tau_acc)*[dbax; dbay; dbaz];
% mag bias
db_mag_dot = - (1/tau_mag)*[dbmx; dbmy; dbmz];
% baro bias
db_baro_dot = - (1/tau_baro)*dbbaro;

%% ========== 6) 에러 상태(22차) 미분방정식 = f_error_sym ==========
%  앞의 12개: 기존 6DoF 에러
f_error_sym = sym(zeros(22,1));

% (a) dot(dP) = p_dot 선형화 => 일단 p_dot 그대로
f_error_sym(1:3) = p_dot;

% (b) dot(dV) = v_dot 선형화
f_error_sym(4:6) = v_dot;

% (c) dot(dTheta) = dOmega
f_error_sym(7:9) = [dwx; dwy; dwz];

% (d) dot(dOmega) = w_dot 선형화
f_error_sym(10:12) = w_dot;

% (e) bias_dot(10차)
f_error_sym(13:15) = db_gyro_dot; % gyro bias
f_error_sym(16:18) = db_acc_dot;  % accel bias
f_error_sym(19:21) = db_mag_dot;  % mag bias
f_error_sym(22)    = db_baro_dot; % baro bias

% 이제 A_error_cont = d(f_error_sym)/d(deltaX_sym)
A_error_cont = jacobian(f_error_sym, deltaX_sym);
A_error_cont_simpl = simplify(A_error_cont);

%% ========== 7) deltaX=0 치환 => 공칭 근방 선형계수 ==========
A_error_lin = subs(A_error_cont_simpl, ...
    {dpx,dpy,dpz, du,dv,dw, dthx,dthy,dthz, dwx,dwy,dwz, ...
     dbgx,dbgy,dbgz, dbax,dbay,dbaz, dbmx,dbmy,dbmz, dbbaro}, ...
    {0,0,0, 0,0,0, 0,0,0, 0,0,0, ...
     0,0,0, 0,0,0, 0,0,0, 0} );
A_error_lin = simplify(A_error_lin);

%% ========== 8) 이산화 (1차 오일러 근사) ==========
nE = 22;
I_nE = eye(nE);
F_error_sym_22 = I_nE + A_error_lin * dt;
F_error_sym_22 = simplify(F_error_sym_22);

%% ========== 9) 프로세스 잡음 Q(22x22) 계산 예시 ==========
%   - 잡음 항을 어떻게 정의하느냐에 따라 달라짐
%   - 여기서는 'W_s * Q_s * W_s^T' 식으로 단순 가정
%   - 그리고 0~dt 적분해서 Qk ~ \int F * WQW^T * F^T d\tau
%
% (a) Qs_diag: 22개 잡음 스케일 심볼릭
Qs_diag = [
    s_px; s_py; s_pz;   % 위치잡음
    s_u;  s_v;  s_w;    % 속도잡음
    s_thx; s_thy; s_thz; % 자세오차잡음
    s_wx_; s_wy_; s_wz_; % 각속도잡음
    s_dbgx; s_dbgy; s_dbgz;  % gyro bias
    s_dbax; s_dbay; s_dbaz;  % accel bias
    s_dbmx; s_dbmy; s_dbmz;  % mag bias
    s_dbbaro             % baro bias
];
Qs_sym = diag(Qs_diag);

% (b) Ws_sym: 어떤 방식으로 오차상태에 잡음이 적용되는지 설정(예시)
%     예: dV -> 1/m, dOmega -> 1/J, db -> ...
Ws_vec = zeros(22,1,'sym'); 
% 예시로 속도오차(du,dv,dw)에 1/m, 각속도오차(dwx,dwy,dwz)에 1/J, 
% bias는 tau^-1 등등... (실제로 모델에 맞게 설정)
% 여기선 "가짜"로 전부 1 처리
Ws_vec(:) = 1;  % 전부 1, 또는 필요한 곳만 1

Ws_sym = diag(Ws_vec);

% (c) Qm_sym = F(tau)* W*Qs*W^T * F(tau)^T ~ 심볼릭 적분
F_t = F_error_sym_22.';
Qm_sym = F_error_sym_22 * Ws_sym * Qs_sym * Ws_sym * F_t;

Qk_sym = int(Qm_sym, tau, 0, dt);
Qk_sym_simpl = simplify(Qk_sym);

%% ========== 10) 화면 출력 ==========
disp('==== A_error_lin (22x22) ====');
disp(A_error_lin);
disp('==== F_error_sym_22 = I + A*dt ===');
disp(F_error_sym_22);
disp('==== Qk_sym_simpl (22x22) ====');
disp(Qk_sym_simpl);

%% ========== 11) 최종 rename & matlabFunction 생성 ==========
Feskf_22 = F_error_sym_22;
Qeskf_22 = Qk_sym_simpl;

matlabFunction(Feskf_22, 'File', 'calcFeskf_22_auto.m', ...
    'Vars', { ...
       px_hat, py_hat, pz_hat, ...
       u_hat, v_hat, w_hat, ...
       qw_hat, qx_hat, qy_hat, qz_hat, ...
       wx_hat, wy_hat, wz_hat, ...
       Fa_x, Fa_y, Fa_z, ...
       Ft_x, Ft_y, Ft_z, ...
       Fr_x, Fr_y, Fr_z, ...
       Fn_x, Fn_y, Fn_z, ...
       m, g, ...
       Ma_x, Ma_y, Ma_z, ...
       Mr_x, Mr_y, Mr_z, ...
       Mn_x, Mn_y, Mn_z, ...
       Cpx, ...
       Jx, Jy, Jz, ...
       tau_gyro, tau_acc, tau_mag, tau_baro, ...
       dt ...
    } ...
);

matlabFunction(Qeskf_22, 'File', 'calcQeskf_22_auto.m', ...
    'Vars', { ...
       px_hat, py_hat, pz_hat, ...
       u_hat, v_hat, w_hat, ...
       qw_hat, qx_hat, qy_hat, qz_hat, ...
       wx_hat, wy_hat, wz_hat, ...
       Fa_x, Fa_y, Fa_z, ...
       Ft_x, Ft_y, Ft_z, ...
       Fr_x, Fr_y, Fr_z, ...
       Fn_x, Fn_y, Fn_z, ...
       m, g, ...
       Ma_x, Ma_y, Ma_z, ...
       Mr_x, Mr_y, Mr_z, ...
       Mn_x, Mn_y, Mn_z, ...
       Cpx, ...
       Jx, Jy, Jz, ...
       tau_gyro, tau_acc, tau_mag, tau_baro, ...
       dt, ...
       s_px, s_py, s_pz, ...
       s_u, s_v, s_w, ...
       s_thx, s_thy, s_thz, ...
       s_wx_, s_wy_, s_wz_, ...
       s_dbgx, s_dbgy, s_dbgz, ...
       s_dbax, s_dbay, s_dbaz, ...
       s_dbmx, s_dbmy, s_dbmz, ...
       s_dbbaro ...
    } ...
);

disp('Done! ==> "calcFeskf_22_auto.m" & "calcQeskf_22_auto.m" generated');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ========== 보조함수들 ==========
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function R = quat_to_DCM(qw,qx,qy,qz)
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

function q_out = quatMultiply(q1, q2)
    qw1=q1(1); qx1=q1(2); qy1=q1(3); qz1=q1(4);
    qw2=q2(1); qx2=q2(2); qy2=q2(3); qz2=q2(4);

    qw = qw1*qw2 - qx1*qx2 - qy1*qy2 - qz1*qz2;
    qx = qw1*qx2 + qx1*qw2 + qy1*qz2 - qz1*qy2;
    qy = qw1*qy2 - qx1*qz2 + qy1*qw2 + qz1*qx2;
    qz = qw1*qz2 + qx1*qy2 - qy1*qx2 + qz1*qw2;

    q_out = [qw; qx; qy; qz];
end

function Om = omegaMat(wx, wy, wz)
    Om = [
       0,   -wx,  -wy,  -wz;
       wx,    0,   wz,  -wy;
       wy,  -wz,    0,   wx;
       wz,   wy,  -wx,    0
    ];
end
