%% ==============================================================
% hanul_sym2disc_euler_aero_improved.m
%  6‑DOF 로켓 (Quaternion) 연속식 → 다양한 이산화 방법 제공
%  ‑‑ 공력 1차 편미분(CXu·CYv·CZw·Clp·Cmq·Cnr) 포함 버전 ‑‑
%  ‑‑ 1차 테일러(기존), 2차 테일러, ZOH 이산화 지원 ‑‑
%  * 모든 과정 disp 출력 *
% ==============================================================

clear all; close all; clc;

import sym.*

fprintf('\n[1] 심볼릭 변수 정의 ---------------------------------------\n')
% ── 상태 ───────────────────────────────────────────────────────
syms x y z  u v w  qw qx qy qz  p q r                real
X = [x;y;z;  u;v;w;  qw;qx;qy;qz;  p;q;r];            % 13×1
disp('X ='); disp(X.');

% ── 입력(총합 힘·모멘트) ───────────────────────────────────────
syms Fx Fy Fz  Mx My Mz                               real
U = [Fx;Fy;Fz;  Mx;My;Mz];                            % 6×1
disp('U ='); disp(U.');

% ── 액추에이터 입력 변수 (9개 - 추력 추가) ──────────────────────────
syms F_thrust d1 d2 d3 d4 T1 T2 T3 T4        real    % 추력, 카나드 각도 & RCS 상태

% ── 파라미터 ─────────────────────────────────────────────────
syms m Jxx Jyy Jzz                                    real positive
J = diag([Jxx Jyy Jzz]);
assume(qw^2+qx^2+qy^2+qz^2 == 1)                      % unit quat

% ── 공력 1차 미분·기준값 ------------------------------------
syms CXu CYv CZw  Clp Cmq Cnr                         real   % 계수 미분
syms qbar Sref Lref                                   real   % 동압·기준치

% ── 추력 파라미터 추가 -------------------------------------
syms F_T                                              real   % 추력값

fprintf('\n[2] DCM (Body→Inertial) -----------------------------------\n')
R11 = qw^2+qx^2-qy^2-qz^2;  R12 = 2*(qx*qy-qw*qz);  R13 = 2*(qx*qz+qw*qy);
R21 = 2*(qx*qy+qw*qz);      R22 = qw^2-qx^2+qy^2-qz^2; R23 = 2*(qy*qz-qw*qx);
R31 = 2*(qx*qz-qw*qy);      R32 = 2*(qy*qz+qw*qx);  R33 = qw^2-qx^2-qy^2+qz^2;
R = [R11 R12 R13; R21 R22 R23; R31 R32 R33];
disp(R);

fprintf('\n[3] 연속‑시간 상태식 f(x,u) --------------------------------\n')
omega   = [p;q;r];
% 3‑1 위치·속도·자세 기초
pos_dot = R*[u;v;w];

% ── 공력 1차 선형 모델 (Body) ────────────────────────────────
F_aero = qbar*Sref*[ CXu*u;  CYv*v;  CZw*w ];
M_aero = qbar*Sref*Lref*[ Clp*p;  Cmq*q;  Cnr*r ];

% 총합 힘·모멘트 (제어 + 공력)
F_tot  = [Fx;Fy;Fz] + F_aero;
M_tot  = [Mx;My;Mz] + M_aero;

% 3‑2 선속·자세·각속도 미분
vel_dot  = (1/m)*F_tot - cross(omega,[u;v;w]);
Omega    = [0 -p -q -r; p 0 r -q; q -r 0 p; r q -p 0];
quat_dot = 0.5*Omega*[qw;qx;qy;qz];  % 중요: 0.5 계수 확인
ang_dot  = J \ (M_tot - cross(omega,J*omega));

f = [pos_dot; vel_dot; quat_dot; ang_dot];
disp(f);

fprintf('\n[4] 자코비안 A,B -------------------------------------------\n')
A = jacobian(f,X);   % 공력 항 포함
fprintf('A 행렬 크기: %d × %d\n', size(A,1), size(A,2));

% ───────────────────────────────────────────────────────────────
%  ▣  B 행렬 "총합 힘·모멘트 → 액추에이터 입력" 변환 ▣
B_FM = jacobian(f,U);              % 13×6 (힘·모멘트 기준)
fprintf('B_FM 행렬 크기(힘/모멘트 기준): %d × %d\n', size(B_FM,1), size(B_FM,2));

%  액추에이터(F_thrust, δ₁–δ₄, T₁–T₄) → 힘·모멘트 매핑 Γ(6×9)
syms d_l d_r T_RCS CL1 CL2 CL3 CL4 real        % 모멘트암·반경·추력·양력계수

% 수정된 Gamma 행렬 - 추력 제어 추가 (첫번째 열)
Gamma = [ ...
    F_T 0 0 0 0  0        0        0        0 ;              % Fx (추력/질량 추가)
    0 0 0 0 0  0        0        0        0 ;                  % Fy
    0 0 0 0 0  0        0        0        0 ;                  % Fz
    0 0 0 0 0  T_RCS*d_r  -T_RCS*d_r  T_RCS*d_r  -T_RCS*d_r ;  % Mx (롤: RCS만)
    0 0 -d_l*CL2 0 d_l*CL4 0 0 0 0 ;                           % My (피치: 2,4만)
    0 d_l*CL1 0 -d_l*CL3 0 0 0 0 0 ];                          % Mz (요: 1,3만)

fprintf('Gamma 행렬 크기(제어 할당): %d × %d\n', size(Gamma,1), size(Gamma,2));

B = B_FM * Gamma;                % 13×9
fprintf('B 행렬 크기(최종 제어 입력): %d × %d\n', size(B,1), size(B,2));
% ───────────────────────────────────────────────────────────────

dt = sym('dt','positive');  % 표본주기

% ===== 이산화 방법 1: 1차 테일러 근사 (기존 방식) =====
fprintf('\n[5.1] 1차 테일러 근사 이산화 (Forward Euler) ---------------\n')
Ad_euler = eye(size(A)) + A*dt;
Bd_euler = B*dt;
fprintf('Ad_euler 크기: %d × %d\n', size(Ad_euler,1), size(Ad_euler,2));
fprintf('Bd_euler 크기: %d × %d\n', size(Bd_euler,1), size(Bd_euler,2));

% ===== 이산화 방법 2: 2차 테일러 근사 (개선) =====
fprintf('\n[5.2] 2차 테일러 근사 이산화 ------------------------------\n')
Ad_taylor2 = eye(size(A)) + A*dt + (A^2)*(dt^2)/2;
Bd_taylor2 = B*dt + A*B*(dt^2)/2;
fprintf('Ad_taylor2 크기: %d × %d\n', size(Ad_taylor2,1), size(Ad_taylor2,2));
fprintf('Bd_taylor2 크기: %d × %d\n', size(Bd_taylor2,1), size(Bd_taylor2,2));

% ===== 이산화 방법 3: 4차 테일러 근사 (고정밀) =====
fprintf('\n[5.3] 4차 테일러 근사 이산화 ------------------------------\n')
Ad_taylor4 = eye(size(A)) + A*dt + (A^2)*(dt^2)/2 + (A^3)*(dt^3)/6 + (A^4)*(dt^4)/24;
Bd_taylor4 = B*dt + A*B*(dt^2)/2 + (A^2)*B*(dt^3)/6 + (A^3)*B*(dt^4)/24;
fprintf('Ad_taylor4 크기: %d × %d\n', size(Ad_taylor4,1), size(Ad_taylor4,2));
fprintf('Bd_taylor4 크기: %d × %d\n', size(Bd_taylor4,1), size(Bd_taylor4,2));

% ===== 이산화 방법 4: 행렬 지수 기반 ZOH (정확한 방법) =====
% 참고: 실제 구현에서는 expm을 사용해야 하지만 심볼릭에서는 테일러 급수로 근사
fprintf('\n[5.4] 행렬 지수 기반 ZOH 이산화 (테일러 급수 근사) ---------\n')
% 행렬 지수 근사 (4차 테일러)
Ad_zoh = eye(size(A)) + A*dt + (A^2)*(dt^2)/2 + (A^3)*(dt^3)/6 + (A^4)*(dt^4)/24;
% 입력 행렬에 대한 ZOH 효과 (∫e^(At)·B·dt의 테일러 급수 근사)
Bd_zoh = (eye(size(A))*dt + A*(dt^2)/2 + (A^2)*(dt^3)/6 + (A^3)*(dt^4)/24) * B;
fprintf('Ad_zoh 크기: %d × %d\n', size(Ad_zoh,1), size(Ad_zoh,2));
fprintf('Bd_zoh 크기: %d × %d\n', size(Bd_zoh,1), size(Bd_zoh,2));

fprintf('[6] 선형화 함수 파일 생성 ------------------------------\n');
% • 입력: state X_trim(13), 입력 U_trim(6), dt, 파라미터 구조체 P
% • 출력: Ad(13×13), Bd(13×9)
varsP = {m, Jxx, Jyy, Jzz, ...                   % 질량·모멘트
         CXu, CYv, CZw, Clp, Cmq, Cnr, ...       % 공력 미분계수
         qbar, Sref, Lref, ...                   % 공력 기준 값
         d_l, d_r, T_RCS, CL1, CL2, CL3, CL4, F_T}; % 액추에이터 파라미터, 추력 추가

% 1. 기존 1차 테일러 근사 함수 (기존과 호환성 유지)
matlabFunction(Ad_euler, Bd_euler, 'Vars', {X, U, dt, varsP{:}}, ...
               'File', 'hanul_linear_AB', 'Optimize', true);

% 2. 2차 테일러 근사 함수
matlabFunction(Ad_taylor2, Bd_taylor2, 'Vars', {X, U, dt, varsP{:}}, ...
               'File', 'hanul_linear_AB_taylor2', 'Optimize', true);

% 3. 4차 테일러 근사 함수 
matlabFunction(Ad_taylor4, Bd_taylor4, 'Vars', {X, U, dt, varsP{:}}, ...
               'File', 'hanul_linear_AB_taylor4', 'Optimize', true);

% 4. ZOH 근사 함수
matlabFunction(Ad_zoh, Bd_zoh, 'Vars', {X, U, dt, varsP{:}}, ...
               'File', 'hanul_linear_AB_zoh', 'Optimize', true);

fprintf('완료! 다음 파일들이 생성되었습니다:\n');
fprintf('  1. hanul_linear_AB.m (1차 테일러 - 기존)\n');
fprintf('  2. hanul_linear_AB_taylor2.m (2차 테일러 - 개선)\n');
fprintf('  3. hanul_linear_AB_taylor4.m (4차 테일러 - 고정밀)\n');
fprintf('  4. hanul_linear_AB_zoh.m (ZOH 근사 - 최고정밀)\n');
fprintf('\n사용 예시:\n');
fprintf('  [Ad,Bd] = hanul_linear_AB_taylor2(X_trim, U_trim, dt, P...);\n');
fprintf('  행렬 크기 - Ad: 13×13, Bd: 13×9\n');  % Bd 크기 변경됨

% linearize_rocket.m 수정 예시 출력
fprintf('\n[7] linearize_rocket.m 수정 예시 ------------------------\n');
fprintf('function [Ad, Bd, A, B, eig_vals] = linearize_rocket(X, final_cmd, params, dt, method)\n');
fprintf('  %% method: 이산화 방법 (''euler'', ''taylor2'', ''taylor4'', ''zoh'')\n');
fprintf('  if nargin < 5, method = ''taylor2''; end  %% 기본값: 2차 테일러\n\n');
fprintf('  %% 선형화 방법에 따라 다른 함수 호출\n');
fprintf('  switch lower(method)\n');
fprintf('    case ''euler''\n');
fprintf('      [Ad, Bd] = hanul_linear_AB(X, U_FM, dt, ...);\n');
fprintf('    case ''taylor2''\n');
fprintf('      [Ad, Bd] = hanul_linear_AB_taylor2(X, U_FM, dt, ...);\n');
fprintf('    case ''taylor4''\n');
fprintf('      [Ad, Bd] = hanul_linear_AB_taylor4(X, U_FM, dt, ...);\n');
fprintf('    case ''zoh''\n');
fprintf('      [Ad, Bd] = hanul_linear_AB_zoh(X, U_FM, dt, ...);\n');
fprintf('    otherwise\n');
fprintf('      error(''Unknown discretization method'');\n');
fprintf('  end\n');
fprintf('end\n');