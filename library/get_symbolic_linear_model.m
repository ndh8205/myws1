function [A_sym, B_sym] = get_symbolic_linear_model(params)
    % 심볼릭 변수를 정의
    syms r1 r2 u v ppsi r m I L T
    syms U1 U2 U3 U4 U5 U6 U7 U8 U9

    % 상태 변수와 입력 벡터 정의
    X_sym = [ r1; r2; u; v; ppsi; r ];
    U_sym = [ U1; U2; U3; U4; U5; U6; U7; U8; U9 ];

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
    x1_dot = [ u * cos(ppsi) - v * sin(ppsi);
               u * sin(ppsi) + v * cos(ppsi) ];

    x2_dot = [ Fx / m - r * v;
               Fy / m + r * u ];

    x3_dot = r;

    x4_dot = Tau / I;

    % 상태 변화율 벡터 구성
    xdot = [ x1_dot; x2_dot; x3_dot; x4_dot ];

    % 상태 변수 벡터
    X_vec = [r1; r2; u; v; ppsi; r];

    % A와 B 행렬 계산
    A_sym = jacobian(xdot, X_vec);
    B_sym = jacobian(xdot, U_sym);

    % 필요한 파라미터를 대입
    A_sym = subs(A_sym, [L, T], [params.Airbearing.D_ref, params.Airbearing.Thrust]);
    B_sym = subs(B_sym, [L, T], [params.Airbearing.D_ref, params.Airbearing.Thrust]);

    % 심볼릭 변수 간소화
    A_sym = simplify(A_sym);
    B_sym = simplify(B_sym);
end