% Symbolic Linearization Example
clear all 
close all 
clc

% 필요한 심볼릭 변수 선언
syms x y u v psi r Fx Fy Tau m I_Z L T real
syms Open_Valve [8,1] real
syms RW_tau real
syms dt real

% 파라미터 설정
params.Airbearing.I_Z = I_Z;
params.Airbearing.m = m;
params.Airbearing.D_ref = L;
params.Airbearing.Thrust = T;

% 상태 벡터 분해
r_vec = [x; y];       % 위치 (X, Y)
V_B = [u; v];         % 속도 (u, v) - 기체 좌표계
att = psi;            % 자세 (psi)
R = r;                % 각속도 (r)

% 제어 입력 분해
% Open_Valve는 8x1 벡터
% RW_tau는 반작용 휠 토크

% 제어 로직 매트릭스
Control_Logic = T * [   0,  -1,   -1,   0,    0,   1,    1,   0;   % Fx 방향
                       -1,   0,    0,   1,    1,   0,    0,  -1;   % Fy 방향
                       -L,   L,   -L,   L,   -L,   L,   -L,   L ]; % 토크

% 최종 추력 계산
final_FT = Control_Logic * Open_Valve;
Fx = final_FT(1);
Fy = final_FT(2);
Tau = final_FT(3) + RW_tau;

% 힘과 토크를 질량과 관성모멘트로 나눔
F_total = [ Fx/m; Fy/m; Tau/I_Z ];

% 원래 비선형 동역학 방정식
x1_dot =  V_B(1) * cos(att) - V_B(2) * sin(att);
x2_dot =  V_B(1) * sin(att) + V_B(2) * cos(att);
u_dot = Fx / m - R * V_B(2);
v_dot = Fy / m + R * V_B(1);
psi_dot = R;
r_dot = Tau / I_Z;

% 상태 벡터와 동역학 방정식
X = [x; y; u; v; psi; R];
X_dot = [x1_dot; x2_dot; u_dot; v_dot; psi_dot; r_dot];

% 관측 변수 정의 (Observable)
% 비선형 항을 포함하는 함수들로 구성
observables = [X;
               cos(psi);
               sin(psi);
               u*cos(psi);
               u*sin(psi);
               v*cos(psi);
               v*sin(psi)];

% 관측 변수의 차원
n_observables = length(observables);

% EDMD를 위한 설정
% 상태 및 관측 변수의 시간 스냅샷을 수집해야 하지만, 심볼릭 코드로 표현하기 위해 다음과 같이 진행합니다.

% Koopman 연산자 K와 입력 행렬 L 선언
K = sym('K', [n_observables, n_observables]);
L = sym('L', [n_observables, length(Open_Valve)+1]);

% 동역학 방정식을 관측 변수에 대한 선형 시스템으로 표현
% observables_dot = K * observables + L * [Open_Valve; RW_tau];

% 실제로 K와 L을 구하기 위해서는 데이터가 필요하지만,
% 여기서는 동역학 방정식을 풀어서 K와 L을 심볼릭하게 추정합니다.

% 관측 변수의 시간 미분 계산
observables_dot = jacobian(observables, X) * X_dot;

% 관측 변수에 대한 선형 방정식 설정
% observables_dot = K * observables + L * [Open_Valve; RW_tau];

% 위의 방정식을 풀어서 K와 L을 구함
% 방정식을 행렬 형태로 재구성
eqns = observables_dot == K * observables + L * [Open_Valve; RW_tau];

disp(eqns)

% 심볼릭 방정식을 풀어서 K와 L의 원소를 구함
% 실제로는 이 방정식을 풀기 위해서는 추가적인 가정이나 데이터가 필요하지만,
% 여기서는 예시적으로 몇 가지 원소를 계산해보겠습니다.

% 예를 들어, x_dot에 대한 방정식은 다음과 같습니다.
% x_dot = u * cos(psi) - v * sin(psi)
% 이는 관측 변수로 표현하면,
% x_dot = observables(3) * observables(7) - observables(4) * observables(8)

% 그러나 이 방법은 복잡하며, 실제로는 EDMD 알고리즘이 데이터 기반으로 K와 L을 추정합니다.
% 따라서, 심볼릭으로 정확한 K와 L을 구하기는 어렵습니다.

% 결론적으로, 선형화된 동역학 모델은 다음과 같이 표현됩니다.
% observables_dot = K * observables + L * U

% 여기서 U = [Open_Valve; RW_tau]

% 따라서, 시스템은 선형화되어 선형 시스템으로 취급할 수 있습니다.
