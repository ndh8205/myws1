clear all
clc
close all

% Define symbolic variables
syms psi tau u v r sigma_x sigma_y sigma_u sigma_v sigma_psi sigma_t dt M Iz T D sigma_Fx sigma_Fy sigma_torque
syms X Y Rp theta real  % 상대 거리와 각도 (로컬 극좌표계)

RB2I_1 = [ cos(theta), -sin(theta); sin(theta), cos(theta) ];
RB2I_2 = [ cos(psi), -sin(psi); sin(psi), cos(psi) ];

% Define the first matrix
A = [ 1  0  cos(psi)*tau  -sin(psi)*tau  (-u*sin(psi)-v*cos(psi))*tau       0;
      0  1  sin(psi)*tau   cos(psi)*tau   (u*cos(psi)-v*sin(psi))*tau       0;
      0  0             1         -r*tau                             0  -v*tau;
      0  0         r*tau              1                             0   u*tau;
      0  0             0              0                             1     tau;
      0  0             0              0                             0       1 ];


% Define the second matrix (diagonal matrix)
Q = diag([sigma_x sigma_y sigma_u sigma_v sigma_psi sigma_t]);

Qf = diag([0 0 sigma_Fx sigma_Fy 0 sigma_torque]);

% Define the third matrix
AT = [                           1                            0       0      0    0  0;
                                 0                            1       0      0    0  0;
                      cos(psi)*tau                 sin(psi)*tau       1  r*tau    0  0;
                     -sin(psi)*tau                 cos(psi)*tau  -r*tau      1    0  0;
      (-u*sin(psi)-v*cos(psi))*tau  (u*cos(psi)-v*sin(psi))*tau       0      0    1  0;
                                 0                            0  -v*tau  u*tau  tau  1 ];


B = [
            0,          0,           0,          0,           0,          0,           0,          0       0;
            0,          0,           0,          0,           0,          0,           0,          0       0;
            0,       -T/M,        -T/M,          0,           0,        T/M,         T/M,          0       0;
         -T/M,          0,           0,        T/M,         T/M,          0,           0,       -T/M       0;
            0,          0,           0,          0,           0,          0,           0,          0       0;
    -(D*T)/Iz,   (D*T)/Iz,   -(D*T)/Iz,   (D*T)/Iz,   -(D*T)/Iz,   (D*T)/Iz,   -(D*T)/Iz,   (D*T)/Iz     1/Iz
         ] .* dt;


W = [ 0,     0,   0,   0,   0,    0;
      0,     0,   0,   0,   0,    0;
      0,     0, 1/M,   0,   0,    0;
      0,     0,   0, 1/M,   0,    0;
      0,     0,   0,   0,   0,    0;
      0,     0,   0,   0,   0, 1/Iz ];


WT = [ 0,     0,   0,   0,    0,    0;
       0,     0,   0,   0,    0,    0;
       0,     0,   M,   0,    0,    0;
       0,     0,   0,   M,    0,    0;
       0,     0,   0,   0,    0,    0;
       0,     0,   0,   0,    0,   Iz ];


gam = W * Qf * WT;
% disp(gam)

gam = int(gam, tau, 0, dt);

% disp(gam)


% Perform the matrix multiplication
Qk = A * W * Q * WT * AT;
% Qk = A * W * Q * W' * A';


% disp(Qk)

% Integral Process Noise

M_integrated = int( Qk, tau, 0, dt );

disp( M_integrated )


% % 좌표 변환: 로컬 극좌표계(rho, theta)에서 전역 직교좌표계(dx, dy)로 변환
% dx = Rp * cos(theta);  % 동체 x 방향 상대 거리
% dy = Rp * sin(theta);  % 동체 y 방향 상대 거리
% 
% LPv = RB2I_2*[dx ; dy];
% 
% % 수정된 측정 모델 정의
% h = [LPv(1);   % 전역 x 방향 상대 거리
%      LPv(2);   % 전역 y 방향 상대 거리
%      psi;  % 절대 방향 (요각)
%      r];   % 각속도
% 
% % 상태 벡터 정의
% X = [X; Y; u; v; psi; r];
% 
% % H 행렬 (측정 모델의 자코비안) 계산
% H = jacobian(h, X);
% 
% % H 행렬 출력
% disp('Measurement Model Jacobian (H):')
% disp(H)