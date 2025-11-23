function [U, computation_time, predicted_trajectory] = Controller_NMPC_KJC4(X, X_target, command, max_torque, idle, dt, params, XF)

    persistent u_prev

    % 현재 상태 업데이트


    if isempty(u_prev)

        u_prev = zeros(9, 1);

    end

    tic;
    
    % MPC 매개변수
    Np = 5;  % 예측 지평선
    Nc = 5;   % 제어 지평선

    PosQ = 1;
    VelQ = 1;
    AttQ = 100000;
    RatQ = 100000;

    Tu_R = 1;
    RW_R = 1;

    Qgain = diag([ PosQ, PosQ, VelQ, VelQ, AttQ, RatQ ]);
    Rgain = diag([ Tu_R * zeros(1,8), RW_R ]);
    

    % 가중치 및 목표값 행렬 생성
    Q = []; R = []; Rr = [];


    % 상태 공간 모델 매개변수
    Iz = params.Airbearing.I_Z;   % [kgmm^2]
    D = params.Airbearing.D_ref;   % [mm] (CG to Thruster)
    M = params.Airbearing.m;       % [kg] (Airbearing mass)
    T = params.Airbearing.Thrust;  % [N] (Mean Thrust Force)


    Ad =  [
            1, 0, dt*cos(X(5)), -dt*sin(X(5)),   dt * ( -X(3)*sin(X(5)) - X(4)*cos(X(5)) ),           0;
            0, 1, dt*sin(X(5)),  dt*cos(X(5)),   dt * (  X(3)*cos(X(5)) - X(4)*sin(X(5)) ),           0;
            0, 0,            1,      -dt*X(6),                                           0,  -dt * X(4);
            0, 0,      dt*X(6),             1,                                           0,   dt * X(3);
            0, 0,            0,             0,                                           1,          dt;
            0, 0,            0,             0,                                           0,           1
          ];


    Bd = [
            0,          0,           0,          0,           0,          0,           0,          0       0;
            0,          0,           0,          0,           0,          0,           0,          0       0;
            0,       -T/M,        -T/M,          0,           0,        T/M,         T/M,          0       0;
         -T/M,          0,           0,        T/M,         T/M,          0,           0,       -T/M       0;
            0,          0,           0,          0,           0,          0,           0,          0       0;
    -(D*T)/Iz,   (D*T)/Iz,   -(D*T)/Iz,   (D*T)/Iz,   -(D*T)/Iz,   (D*T)/Iz,   -(D*T)/Iz,   (D*T)/Iz     max_torque/49/Iz
         ] .* dt;

    Cd = eye(6) + eye(6) * dt;

    % 가중치 행렬
    q = eye( size( Cd, 1) ) .* Qgain;  % 상태 추적 오차에 대한 가중치
    r = eye( size( Bd, 2 ) ) .* Rgain;     % 제어 변화에 대한 가중치

    % LQR 해 계산
    % [K_lqr, P, ~] = lqr(Ad, Bd, Q_lqr, R_lqr);

    % % 가중치 행렬 생성
    % Q = repmat(Qgain, [Np, 1]);
    % Q((Np-1)*size(Qgain,1)+1:end, (Np-1)*size(Qgain,2)+1:end) = P;
    % R = repmat(Rgain, [Nc, 1]);
    % Rr = repmat(X_target, [Np, 1]);

    % P = diag([PosQ*10 PosQ*10 VelQ*10 VelQ*10 AttQ*10 RatQ*10]);

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

    % for i = 1 : Np
    % 
    %     W_block = W_1 * ( A_a^( i - 1 ) );
    %     W = [ W; W_block ];
    % 
    % end

    % W 행렬 생성
    W = zeros(p * Np, n + p);
    W_1 = C_a * A_a;
    A_a_power = eye(size(A_a));
    
    for i = 1:Np
        W((i-1)*p+1:i*p, :) = W_1 * A_a_power;
        A_a_power = A_a_power * A_a;
    end


    % Z 행렬 생성
    Z = zeros(p * Np, m * Nc);
    A_a_power = eye(size(A_a));
    C_a_B_a = C_a * B_a;
    
    for i = 1:Np
        row_start = (i-1)*p + 1;
        row_end = i*p;
        for j = 1:min(i, Nc)
            col_start = (j-1)*m + 1;
            col_end = j*m;
            Z(row_start:row_end, col_start:col_end) = C_a * A_a_power * B_a;
        end
        A_a_power = A_a_power * A_a;
    end
     

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

    f = Z'*Q*(W*XF-Rr); % 제곱꼴 1차항에 붙는 행렬

    % 제약 조건

    % U
    u_min = [-inf; -inf; -inf; -inf; -inf; -inf; -inf; -inf; 40];
    u_max = [inf; inf; inf; inf; inf; inf; inf; inf; 58];

    % u_min = [-inf; -inf; -inf; -inf; -inf; -inf; -inf; -inf; -max_torque/6];
    % u_max = [inf; inf; inf; inf; inf; inf; inf; inf; max_torque/6];

    U_min = [];

    for i = 1:Nc

        U_min((i-1)*m+1 : i*m, 1) = u_min;

    end

    U_max = [];

    for i = 1:Nc

        U_max((i-1)*m+1 : i*m, 1) = u_max;

    end

    % U_min = u_min*ones(m*Nc,1);
    % U_max = u_max*ones(m*Nc,1);

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
    
    % % Y
    % y_min = [-inf; -inf; -40; -40; -inf; -inf];
    % y_max = [inf; inf; 40; 40; inf; inf];
    % 
    % Y_min = [];
    % 
    % for i = 1:Np
    % 
    %     Y_min((i-1)*p+1 : i*p, 1) = y_min;
    % 
    % end
    % 
    % Y_max = [];
    % 
    % for i = 1:Np
    % 
    %     Y_max((i-1)*p+1 : i*p, 1) = y_max;
    % 
    % end
    % 
    % C2 = [-Z; Z];
    % 
    % % all con
    % Acon = [C2;
    %         C3];
    % 
    % 
    % C_min_max = [-Y_min + W*XF;
    %              Y_max - W*XF;
    %              -U_min + E*u_prev;
    %              U_max - E*u_prev];


    % U con
    Acon = C3;


    C_min_max = [-U_min + E*u_prev;
                 U_max - E*u_prev];
    
    % Y con
    % Acon = C2;
    % 
    % C_min_max = [ -Y_min + W * XF;
    %              Y_max - W * XF ];

    % 최적화 문제 해결
    options = optimset('Algorithm', 'interior-point-convex', 'Display', 'off');
    [DeltaU, ~, exitflag, ~] = quadprog(H, f, Acon, C_min_max, [], [], [], [], [], options);

    % 최적 제어 입력 추출
    deltau = Nc_block * DeltaU;
    U = u_prev + deltau;

    % 릴레이 제어 (ON-OFF)
    U(1:8) = double( U(1:8) >= mean( U(1:8) ) );

    % 다음 단계를 위해 제어 입력 저장
    u_prev = U;

    computation_time = toc;
    disp(['Computation time: ', num2str(computation_time), ' seconds']);

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