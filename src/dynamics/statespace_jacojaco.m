% MATLAB 코드: 쿼터니언을 포함한 13x13 상태 공간 모델의 심볼릭 자코비안 행렬 생성
% 이 코드는 확장 칼만 필터(EKF)에 사용될 자코비안 행렬을 계산합니다.

% 워크스페이스 초기화 및 커맨드 창 클리어
clear; clc; close all;

% 심볼릭 변수 정의 (13개의 상태 변수)
syms x y z real        % 위치 (X, Y, Z) - 관성 프레임
syms u v w real        % 속도 (u, v, w) - 바디 프레임
syms qw qx qy qz real  % 쿼터니언 (qw, qx, qy, qz)
syms p q r real        % 각속도 (p, q, r) - 바디 프레임

% 상태 벡터 X
X = [x; y; z; u; v; w; qw; qx; qy; qz; p; q; r];

% 심볼릭 변수 정의 (제어 입력 U: 8개의 RCS 스러스터)
syms U1 U2 U3 U4 U5 U6 U7 U8 real
U = [U1; U2; U3; U4; U5; U6; U7; U8];

% 심볼릭 변수 정의 (프로세스 노이즈 Qk)
syms Fx_Q Fy_Q Fz_Q Rx_Q Py_Q Yz_Q real
Qk = [Fx_Q; Fy_Q; Fz_Q; Rx_Q; Py_Q; Yz_Q];

% 시간 및 파라미터 변수 정의
syms t real
% params = Params_init_HANul(); % 심볼릭 계산 시 파라미터는 일반적으로 상수로 처리

% 중력 가속도 (실제 값 사용 시 대체)
syms g real

% 질량 (심볼릭 변수로 정의)
syms m real

% 관성 텐서 (심볼릭 행렬으로 정의, 대칭 가정)
syms Jxx Jxy Jxz Jyx Jyy Jyz Jzx Jzy Jzz real
J = [Jxx, Jxy, Jxz;
     Jyx, Jyy, Jyz;
     Jzx, Jzy, Jzz];

% 바디 프레임에서의 추력 벡터 (심볼릭 변수)
syms F_T1 F_T2 F_T3 real
F_T_body = [F_T1; F_T2; F_T3];

% 쿼터니언 정규화 (단위 쿼터니언 유지)
q_norm = sqrt(qw^2 + qx^2 + qy^2 + qz^2);
qw_norm = qw / q_norm;
qx_norm = qx / q_norm;
qy_norm = qy / q_norm;
qz_norm = qz / q_norm;

% 정규화된 쿼터니언 벡터
q_norm_vec = [qw_norm; qx_norm; qy_norm; qz_norm];

% 회전 행렬 (쿼터니언을 이용하여 바디 프레임에서 관성 프레임으로 변환)
R_B2I = [
    qw_norm^2 + qx_norm^2 - qy_norm^2 - qz_norm^2, 2*(qx_norm*qy_norm - qw_norm*qz_norm), 2*(qx_norm*qz_norm + qw_norm*qy_norm);
    2*(qx_norm*qy_norm + qw_norm*qz_norm), qw_norm^2 - qx_norm^2 + qy_norm^2 - qz_norm^2, 2*(qy_norm*qz_norm - qw_norm*qx_norm);
    2*(qx_norm*qz_norm - qw_norm*qy_norm), 2*(qy_norm*qz_norm + qw_norm*qx_norm), qw_norm^2 - qx_norm^2 - qy_norm^2 + qz_norm^2
];

% 관성 프레임에서 바디 프레임으로의 회전 행렬
R_I2B = transpose(R_B2I);

% 중력 힘을 바디 프레임으로 변환
F_G = R_I2B * [0; 0; m*g];

% 총 힘 (바디 프레임)
F_total = F_T_body + F_G + [Fx_Q; Fy_Q; Fz_Q];

% 각속도 벡터 정의
omega = [p; q; r];

% 위치의 미분 (관성 프레임에서의 속도)
x1_dot = R_B2I * [u; v; w];

% 속도의 미분 (바디 프레임에서의 가속도)
x2_dot = (F_total / m) - cross(omega, [u; v; w]);

% 쿼터니언의 미분 (관성 프레임)
Omega = [
    0,    -p, -q, -r;
    p,     0,  r, -q;
    q,    -r,  0,  p;
    r,     q, -p,  0
];
q = q_norm_vec;
x3_dot = 0.5 * Omega * q;

% 각속도의 미분 (바디 프레임)
% 여기서는 공기역학적 토크를 포함하지 않고, 프로세스 노이즈만 고려
M_total = [Rx_Q; Py_Q; Yz_Q];
x4_dot = J \ (M_total - cross(omega, J*omega));

% 전체 상태의 미분 벡터
xdot = [x1_dot; x2_dot; x3_dot; x4_dot];

% 자코비안 행렬 F = df/dX 계산
F = jacobian(xdot, X);

% 심볼릭 자코비안 행렬 출력
disp('심볼릭 자코비안 행렬 A:');
disp(F);