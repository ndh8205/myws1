% MATLAB 코드: 쿼터니언을 포함한 13x13 상태 공간 모델의 심볼릭 자코비안 행렬 생성
% 이 코드는 확장 칼만 필터(EKF)에 사용될 자코비안 행렬을 계산합니다.

% 워크스페이스 초기화 및 커맨드 창 클리어
clear; clc; close all;

% 심볼릭 변수 정의 (13개의 상태 변수)
syms x y z real        % 위치 (X, Y, Z) - 관성 프레임
syms u v w real        % 속도 (u, v, w) - 바디 프레임
syms qw qx qy qz real  % 쿼터니언 (qw, qx, qy, qz)
syms p q r real        % 각속도 (p, q, r) - 바디 프레임
% 주의: 'q'는 각속도 변수와 교착 상태를 피하기 위해 'p_q'로 변경

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
% J = [Jxx, Jxy, Jxz;
%      Jyx, Jyy, Jyz;
%      Jzx, Jzy, Jzz];

J = [Jxx, 0, 0;
     0, Jyy, 0;
     0, 0, Jzz];

% 바디 프레임에서의 추력 벡터 (심볼릭 변수)
syms F_T1 F_T2 F_T3 real
F_T_body = [F_T1; F_T2; F_T3];

% 쿼터니언 정규화 (단위 쿼터니언 유지)
% q_norm = sqrt(qw^2 + qx^2 + qy^2 + qz^2);
% qw_norm = qw / q_norm;
% qx_norm = qx / q_norm;
% qy_norm = qy / q_norm;
% qz_norm = qz / q_norm;
% q_norm = sqrt(qw^2 + qx^2 + qy^2 + qz^2);
qw_norm = qw;
qx_norm = qx;
qy_norm = qy;
qz_norm = qz;

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
    p,    0,  r, -q;
    q,    -r,  0,  p;
    r,     q, -p, 0
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


%% 검증 스크립트 시작

% 테스트 포인트 설정
test_state = zeros(13,1);
test_state(7) = 1;  % 단위 쿼터니언 [1,0,0,0]
test_input = zeros(8,1);
test_noise = zeros(6,1);

% 파라미터 설정
test_params.g = 9.81;
test_params.m = 100;  % 100kg
test_params.J = diag([10, 10, 10]);  % 단순화된 관성텐서
test_params.F_T = [0; 0; 0];  % 추력 없음

% 1. 쿼터니언 정규화 검증
disp('쿼터니언 정규화 검증:')
q_test = [0.5; 0.5; 0.5; 0.5];  % 비정규화된 쿼터니안
q_norm = sqrt(sum(q_test.^2));
q_normalized = q_test / q_norm;
disp(['정규화된 쿼터니언 크기: ', num2str(norm(q_normalized))])

% 2. 회전행렬 검증 (수정된 버전)
% 먼저 수치값을 대입한 후 double로 변환
test_quat = q_normalized;  % [qw, qx, qy, qz]
R_test = [
    test_quat(1)^2 + test_quat(2)^2 - test_quat(3)^2 - test_quat(4)^2, 2*(test_quat(2)*test_quat(3) - test_quat(1)*test_quat(4)), 2*(test_quat(2)*test_quat(4) + test_quat(1)*test_quat(3));
    2*(test_quat(2)*test_quat(3) + test_quat(1)*test_quat(4)), test_quat(1)^2 - test_quat(2)^2 + test_quat(3)^2 - test_quat(4)^2, 2*(test_quat(3)*test_quat(4) - test_quat(1)*test_quat(2));
    2*(test_quat(2)*test_quat(4) - test_quat(1)*test_quat(3)), 2*(test_quat(3)*test_quat(4) + test_quat(1)*test_quat(2)), test_quat(1)^2 - test_quat(2)^2 - test_quat(3)^2 + test_quat(4)^2
];

% 회전행렬 검증
I_check = R_test * R_test';
disp('회전행렬 정직교성 검증 (should be close to identity):')
disp(I_check)
disp(['행렬식: ', num2str(det(R_test))])  % should be 1

% 3. 수치적 자코비안 계산 준비
delta = 1e-6;

% 시스템 동역학 함수 (실제 구현)
function dx = system_dynamics(state)
    % 상태 벡터 분해
    x = state(1:3);   % 위치
    v = state(4:6);   % 속도
    q = state(7:10);  % 쿼터니언
    w = state(11:13); % 각속도
    
    % 쿼터니언 정규화
    q = q / norm(q);
    
    % 회전 행렬 계산
    R = quat2rotm(q');  % MATLAB 내장 함수 사용
    
    % 상태 미분 계산
    dx = zeros(13, 1);
    dx(1:3) = R * v;  % 위치 미분
    dx(4:6) = cross(-w, v);  % 속도 미분
    
    % 쿼터니언 미분
    Omega = [
        0,    -w(1), -w(2), -w(3);
        w(1),  0,     w(3), -w(2);
        w(2), -w(3),  0,     w(1);
        w(3),  w(2), -w(1),  0
    ];
    dx(7:10) = 0.5 * Omega * q;
    
    % 각속도 미분 (간단한 모델)
    dx(11:13) = zeros(3, 1);  % 토크가 없다고 가정
    
    return;
end

% 수치적 자코비안 계산
J_num = zeros(13, 13);
for i = 1:13
    state_plus = test_state;
    state_plus(i) = state_plus(i) + delta;
    state_minus = test_state;
    state_minus(i) = state_minus(i) - delta;
    
    dx_plus = system_dynamics(state_plus);
    dx_minus = system_dynamics(state_minus);
    
    J_num(:,i) = (dx_plus - dx_minus) / (2*delta);
end

disp('수치적 자코비안의 일부:')
disp(J_num(1:4, 1:4))

% 4. 물리적 의미 검증
% 속도-위치 관계 검증
pos_vel_block = J_num(1:3, 4:6);
disp('속도-위치 관계 블록:')
disp(pos_vel_block)

% 각속도-쿼터니언 관계 검증
quat_angular_block = J_num(7:10, 11:13);
disp('각속도-쿼터니언 관계 블록:')
disp(quat_angular_block)

% 5. 구조적 검증
% 자코비안의 영점(zero) 패턴 확인
zero_pattern = abs(J_num) < 1e-10;
disp('자코비안의 구조적 영점 패턴:')
disp(zero_pattern)