function [U, computation_time, predicted_trajectory] = Controller_NMPC_ver1(X, X_target, command, max_torque, idle, dt, params, XF)
    tic;
 
    Np = 50;
    Nc = 50;

    % 가중치 조정
    PosQ = 1;    
    VelQ = 10;     
    AttQ = 1.2;   
    RatQ = 10;

    Qgain = diag([PosQ PosQ VelQ VelQ AttQ RatQ]);
    Rgain = diag([1 1 1 1 1 1 1 1 0.3]);

    % 시스템 모델 매개변수
    I_Z = params.Airbearing.I_Z;
    d = params.Airbearing.D_ref;
    m_A = params.Airbearing.m;
    T = params.Airbearing.Thrust;

    % 상태 공간 모델
    Ac = [
        0,  0,    cos(X(5)),   -sin(X(5)),   0,   0;
        0,  0,    sin(X(5)),    cos(X(5)),    0,   0;
        0,  0,            0,        -X(6),   0,      0;
        0,  0,         X(6),            0,   0,      0;
        0,  0,            0,            0,   0,      1;
        0,  0,            0,            0,   0,      0 ];

    Bc = [
               0,            0,            0,           0,            0,           0,            0,           0,        0;
               0,            0,            0,           0,            0,           0,            0,           0,        0;
               0,       -T/m_A,       -T/m_A,           0,            0,       T/m_A,        T/m_A,           0,        0;
          -T/m_A,            0,            0,       T/m_A,        T/m_A,           0,            0,      -T/m_A,        0;
               0,            0,            0,           0,            0,           0,            0,           0,        0;
      -(d*T)/I_Z,    (d*T)/I_Z,   -(d*T)/I_Z,   (d*T)/I_Z,   -(d*T)/I_Z,   (d*T)/I_Z,   -(d*T)/I_Z,   (d*T)/I_Z,    1/I_Z
    ];

    Cc = eye(6);
    Dc = zeros(length(Cc(:, 1)), length(Bc(1, :)));


    % 이산화
    [Ad, Bd, Cd, Dd] = c2dm(Ac, Bc, Cc, Dc, dt);

    % 증강 시스템 구성
    [n, ~] = size(Ad);
    [~, m] = size(Bd);
    [p, ~] = size(Cd);

    A_a = eye( n + p, n + p );
    A_a( 1 : n, 1 : n ) = Ad;
    A_a( n + 1 : n + p, 1 : n ) = Cd * Ad;
    
    B_a = zeros( n + p, m );
    B_a( 1 : n, : ) = Bd;
    B_a( n + 1 : n + p, : ) = Cd * Bd;
    
    C_a = zeros( p, n + p );
    C_a( :, n + 1 : n + p ) = eye( p, p );

    % 예측 행렬 구성
    W = [];    
    W_1 = C_a * A_a;
    
    for i = 1:Np
        W_block = W_1 * (A_a^(i-1));
        W = [W; W_block];
    end

    % 제어 행렬 구성
    Z = zeros( p * Np, m * Nc );
    
    for i = 1:Np

        for j = 1:Nc

            if i >= j

                Z((i-1)*p+1:i*p, (j-1)*m+1:j*m) = C_a * A_a^(i-j) * B_a;

            else

                Z((i-1)*p+1:i*p, (j-1)*m+1:j*m) = zeros(p, m);

            end

        end

    end

    % 가중치 및 목표값 행렬 생성
    Q = []; R = []; Rr = [];

    for i = 1:Np

        Q = blkdiag(Q, Qgain);
        Rr = [Rr; X_target];

    end    

    for i = 1:Nc

        R = blkdiag(R, Rgain);

    end

    % 제약조건 설정
    y_min = [-inf; -inf; -30; -30; -inf; -inf];
    y_max = [inf; inf; 30; 30; -inf; inf];
    
    u_min = [-inf(8,1); -max_torque];
    u_max = [inf(8,1); max_torque];

    % 제약조건 행렬 구성
    Y_min = repmat(y_min, Np, 1);
    Y_max = repmat(y_max, Np, 1);
    U_min = repmat(u_min, Nc, 1);
    U_max = repmat(u_max, Nc, 1);

    % 상태 제약조건
    C1 = [ -Z; Z ];
    d1 = [ -Y_min + W*XF; Y_max - W*XF ];

    % 입력 제약조건
    C2 = [ -eye(m*Nc); eye(m*Nc) ];
    d2 = [ -U_min; U_max ];

    % 전체 제약조건
    Acon = [C1; C2];
    bcon = [d1; d2];

    % 비용 함수
    H = Z'*Q*Z + R;
    f = Z'*Q*(W*XF - Rr);

    % 최적화 설정
    options = optimset('Algorithm', 'interior-point-convex', ...
                      'Display', 'off', ...
                      'MaxIter', 100, ...
                      'TolFun', 1e-4, ...
                      'TolCon', 1e-4);
    
    % QP 해결
    [DeltaU, ~, exitflag] = quadprog(H, f, [], [], [], [], [], [], [], options);
    
    % 제어 입력 계산
    if exitflag > 0
        U = reshape(DeltaU(1:9), 9, 1);
        
        % 추력기 제어
        U(1:8) = double( U(1:8) >= mean( U(1:8) ) );
        
        % 반작용 휠 스케일링
        % Motor_cmd = U(9);
        % min_input = -93/40676;
        % max_input = 0.005;
        % motor_command = scale_command(Motor_cmd, min_input, max_input, 49);
        % U(9) = LIMIT2(motor_command, 0, 98);
    else
        % 최적화 실패 시 안전한 제어 입력
        U = [zeros(8,1); idle];
    end
    
    % 예측 궤적 계산
    predicted_trajectory = zeros(6, Np+1);
    predicted_trajectory(:,1) = X;
    
    for i = 1:Np
        predicted_trajectory(:,i+1) = Ad * predicted_trajectory(:,i) + Bd * U;
    end
    
    computation_time = toc;
    disp(['Computation time: ', num2str(computation_time), ' seconds']);
end