% Symbolic Linearization Example
clear all 
close all 
clc

% 필요한 심볼릭 변수 정의
syms x1 x2 x3 x4 x5 x6 u1 u2 u3 u4 u5 u6 u7 u8 u9 real

% 파라미터 정의 (일반적으로 상수로 간주)
syms I m L T real

% 상태 벡터
X = [x1; x2; x3; x4; x5; x6];

% 제어 벡터
U = [u1; u2; u3; u4; u5; u6; u7; u8; u9];

% Control Logic 매트릭스
Control_Logic = T .* [   0,  -1,   -1,   0,    0,   1,    1,   0;   % X
                        -1,   0,    0,   1,    1,   0,    0,  -1;   % Y
                        -L,   L,   -L,   L,   -L,   L,   -L,   L ]; % Tau

% Thruster Forces 계산
final_FT = Control_Logic * U(1:8);
Fx = final_FT(1);
Fy = final_FT(2);
Tau = final_FT(3) + U(9);

% F_total 정의
F_total = [ Fx/m; Fy/m; Tau/I ];

% 상태 미분 계산
att = x5;
R = x6;

% V_B 정의
V_B = [x3; x4];

x1_dot = V_B(1) * cos(att) - V_B(2) * sin(att);
x2_dot = V_B(1) * sin(att) + V_B(2) * cos(att);

x3_dot = Fx / m - R * X(4);
x4_dot = Fy / m + R * X(3);

x5_dot = R;
x6_dot = Tau / I;

xdot = [x1_dot; x2_dot; x3_dot; x4_dot; x5_dot; x6_dot];

% Jacobian 행렬 계산
A_sym = jacobian(xdot, X);
B_sym = jacobian(xdot, U);

% 작동점 설정 (예시: X0 및 U0)
% 예를 들어, 정지 상태에서의 작동점
X0 = [0; 0; 0; 0; 0; 0];
U0 = zeros(9,1); % 모든 제어 입력이 0인 경우

% 파라미터 값 설정 (예시 값, 실제 시스템에 맞게 조정 필요)
I_val = 1.0;    % 관성 모멘트 (예시 값)
m_val = 1.0;    % 질량 (예시 값)
L_val = 1.0;    % 참조 길이 (예시 값)
T_val = 1.0;    % 추력 (예시 값)

% A_sym 및 B_sym을 X0, U0 및 파라미터 값으로 대체
subs_A = subs(A_sym, {x1, x2, x3, x4, x5, x6, ...
                       u1, u2, u3, u4, u5, u6, u7, u8, u9, ...
                       I, m, L, T}, ...
                       {X0(1), X0(2), X0(3), X0(4), X0(5), X0(6), ...
                        U0(1), U0(2), U0(3), U0(4), U0(5), U0(6), U0(7), U0(8), U0(9), ...
                        I_val, m_val, L_val, T_val});
                    
subs_B = subs(B_sym, {x1, x2, x3, x4, x5, x6, ...
                       u1, u2, u3, u4, u5, u6, u7, u8, u9, ...
                       I, m, L, T}, ...
                       {X0(1), X0(2), X0(3), X0(4), X0(5), X0(6), ...
                        U0(1), U0(2), U0(3), U0(4), U0(5), U0(6), U0(7), U0(8), U0(9), ...
                        I_val, m_val, L_val, T_val});

% MATLAB 행렬로 변환
A_numeric = double(subs_A);
B_numeric = double(subs_B);

% 결과 출력
disp('A matrix:');
disp(A_numeric);

disp('B matrix:');
disp(B_numeric);
