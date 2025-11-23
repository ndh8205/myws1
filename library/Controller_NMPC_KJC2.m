function [U, computation_time, predicted_trajectory] = Controller_NMPC_KJC2(X, X_target, dt, params, XF)
    
    % MPC 매개변수
    Np = 15;  % 예측 지평선
    Nc = 15;   % 제어 지평선

    PosQ = 1;
    VelQ = 1;
    AttQ = 100000;
    RatQ = 1000;

    Qgain = diag([ PosQ PosQ VelQ VelQ AttQ RatQ]);
    Rgain = diag([ 1 1 1 1 1 1 1 0.1 200]);


    % Qgain = 1;

    % 상태 공간 모델 매개변수
    I_Z = params.Airbearing.I_Z;   % [kgmm^2]
    d = params.Airbearing.D_ref;   % [mm] (CG to Thruster)
    m_A = params.Airbearing.m;       % [kg] (Airbearing mass)
    T = params.Airbearing.Thrust;  % [N] (Mean Thrust Force)

    A = [

        0, 0, cos(X(5)), -sin(X(5)), - X(3)*sin(X(5))-X(4)*cos(X(5)), 0;
        0, 0, sin(X(5)),  cos(X(5)),   X(3)*cos(X(5))-X(4)*sin(X(5)), 0;
        0, 0,        0,        -X(6),                      0, -X(4);
        0, 0,        X(6),         0,                      0, X(3);
        0, 0,        0,         0,                      0, 1;
        0, 0,        0,         0,                      0, 0

        ];


    % A의 거듭제곱 계산
    A2 = A^2;
    A3 = A^3;
    A4 = A^4;

    % 4차까지의 테일러 급수 전개
    Ad = eye(6) + A*dt + (A2*dt^2)/2 + (A3*dt^3)/6 + (A4*dt^4)/24;

    Bc = [0      0      0      0      0      0      0      0     0;
          0      0      0      0      0      0      0      0     0;
          0    -T/m_A   -T/m_A     0      0     T/m_A    T/m_A     0     0;
        -T/m_A     0      0     T/m_A    T/m_A     0      0    -T/m_A    0;
          0      0      0      0      0      0      0      0      0;
        -T*d/I_Z   T*d/I_Z   -T*d/I_Z    T*d/I_Z   -T*d/I_Z    T*d/I_Z   -T*d/I_Z    T*d/I_Z     1/I_Z ];


    Cc = eye(6);

    

    Bd = Bc * dt;


    Cd = eye(6) + Cc * dt;

    % 가중치 행렬
    q = eye( size( Cd, 1) ) .* Qgain;  % 상태 추적 오차에 대한 가중치
    r = eye( size( Bd, 2 ) ) .* Rgain;     % 제어 변화에 대한 가중치

    % 증강된 상태 공간 모델
    [ n, ~ ] = size( Ad );
    [ ~, m ] = size( Bd );
    [ p, ~ ] = size( Cd );

    A_a = eye( n + p, n + p );
    A_a( 1 : n, 1 : n ) = Ad;
    A_a( n + 1 : n + p, 1 : n ) = Cd * Ad;
    
    B_a = zeros( n + p, m );
    B_a( 1 : n, : ) = Bd;
    B_a( n + 1 : n + p, : ) = Cd * Bd;
    
    C_a = zeros( p, n + p );
    C_a( :, n + 1 : n + p ) = eye( p, p );

    % W 행렬 생성
    W = [];    
    W_1 = C_a * A_a;

    tic;

    for i = 1 : Np

        W_block = W_1 * ( A_a^( i - 1 ) );
        W = [ W; W_block ];

    end

    % Z 행렬 생성
    Z = zeros( p * Np, m * Nc );

    for i = 1:Np

        for j = 1:Nc

            if i >= j

                Z( ( i - 1 ) * p + 1 : i * p, ( j - 1 ) * m + 1 : j * m) = C_a * A_a^( i - j ) * B_a;

            else

                Z( ( i - 1 ) * p + 1 : i * p, ( j - 1 ) * m + 1 : j * m ) = zeros( p, m );

            end

        end

    end
    

    % 가중치 및 목표값 행렬 생성
    Q = []; R = []; Rr = [];

    for i = 1:Np

      Q = blkdiag( Q, q );
      Rr = [ Rr; X_target ];

    end    

    for i = 1:Nc

      R = blkdiag( R, r );
      
    end

    % Hessian 및 기울기 벡터 계산
    H = Z' * Q * Z + R;
    Nc_block = [ eye( m, m ), zeros( m, m * Nc - m ) ];

    % 현재 상태 업데이트

    persistent u_prev

    if isempty(u_prev)

        u_prev = zeros(m, 1);

    end

    f = Z' * Q * ( W * XF - Rr );

    % 제약 조건

    u_min = [zeros(8,1); -566900/2];  % 최소 토크
    u_max = [ones(8,1); 566900/2];   % 최대 토크

    U_min = [];

    for i = 1:Nc

        U_min( ( i - 1 ) * m + 1 : i * m, 1 ) = u_min;

    end

    U_max = [];

    for i = 1:Nc

        U_max( ( i - 1 ) * m + 1 : i * m, 1 ) = u_max;

    end

    C3_H = zeros(Nc*m,Nc*m);

    for i = 1:Nc
        for j = 1:Nc
            if i >= j
                C3_H((i-1)*m + 1:i*m, (j-1)*m + 1:j*m) = eye(m);
            end
        end
    end

    E = [];
    for i = 1:Nc
        E((i-1)*m+1 : i*m, 1:m) = eye(m);
    end

    C3 = [-C3_H; C3_H];

    
    % Y
    y_min = [ -inf; -inf; -30; -40; -inf; -inf ];
    y_max = [ inf; inf; 40; 40; inf; inf ];

    Y_min = [];

    for i = 1:Np

        Y_min( ( i - 1 ) * p + 1 : i * p, 1 ) = y_min;

    end

    Y_max = [];

    for i = 1:Np

        Y_max( ( i - 1 ) * p + 1 : i * p, 1 ) = y_max;

    end

    C2 = [ -Z; Z ];

    % % Y con
    % Acon = C2;
    % 
    % C_min_max = [ -Y_min + W * XF;
    %              Y_max - W * XF ];

    % all con
    Acon = [C2;
            C3];


    C_min_max = [-Y_min + W*XF;
                 Y_max - W*XF;
                 -U_min + E*u_prev;
                 U_max - E*u_prev];

    % 최적화 문제 해결
    options = optimset( 'Algorithm', 'interior-point-convex', 'Display', 'off' );
    [DeltaU, ~, exitflag, ~] = quadprog( H, f, Acon, C_min_max, [], [], [], [], [], options );
    computation_time = toc;
    disp(['Computation time: ', num2str(computation_time), ' seconds']);
    debugU = DeltaU;
    disp(DeltaU(1:9))

    % 최적 제어 입력 추출
    deltau = DeltaU(1:9);
    U = u_prev + deltau;

    % 릴레이 제어 (ON-OFF)
    U(1:8) = double( U( 1:8 ) >= mean( U( 1:8 ) ) );
    % U(9) = cmd2tq( U(9), 49, params, dt );

    % 다음 단계를 위해 제어 입력 저장
    u_prev = U;

    % computation_time = toc;
    % disp(['Computation time: ', num2str(computation_time), ' seconds']);

    % 예측 궤적 계산
    predicted_trajectory = zeros(6, Np+1);
    predicted_trajectory(:,1) = X;

    for i = 1:Np
        if i <= Nc
            u_k = U;
        else
            u_k = U(:,end);
        end
        predicted_trajectory(:,i+1) = Ad * predicted_trajectory(:,i) + Bd * u_k;
    end
end