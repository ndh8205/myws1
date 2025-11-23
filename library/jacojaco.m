clear all
clc
close all

% 심볼릭 변수 정의
syms r1 r2 u v psi r m I L T dt
syms U1 U2 U3 U4 U5 U6 U7 U8 U9

% 상태 벡터 X와 제어 입력 U 정의
X = [ r1; r2; u; v; psi; r ];
U = [ U1; U2; U3; U4; U5; U6; U7; U8; U9 ];

% 제어 로직 설정
Open_Valve = [U1; U2; U3; U4; U5; U6; U7; U8];
RW_tau = U9;  % 리액션 휠 토크를 U9로 간주

% 제어 로직 행렬 정의
Control_Logic = T .* [   0,  -1,   -1,   0,    0,   1,    1,   0;   % X
                        -1,   0,    0,   1,    1,   0,    0,  -1;   % Y
                        -L,   L,   -L,   L,   -L,   L,   -L,   L ]; % Tau

% 최종 추력 및 모멘트 계산
final_FT = Control_Logic * Open_Valve;
Fx = final_FT(1);
Fy = final_FT(2);
Tau = final_FT(3) + RW_tau;

% 동역학 방정식 정의
x1_dot = [ u * cos(psi) - v * sin(psi);
           u * sin(psi) + v * cos(psi) ];

x2_dot = [ Fx / m - r * v;
           Fy / m + r * u ];

x3_dot = r;

x4_dot = Tau / I;

% 상태 변화율 벡터 구성
xdot = [ x1_dot; x2_dot; x3_dot; x4_dot ];

% Jacobian 행렬 계산
A = jacobian(xdot, X);
B = jacobian(xdot, U);

% 간소화 (필요시)
A = simplify(A);
B = simplify(B);

% A와 B 행렬 출력
disp('Matrix A:');
disp(A);

disp('Matrix B:');
disp(B);
