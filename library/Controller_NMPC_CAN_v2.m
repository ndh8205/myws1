function [U, computation_time, predicted_trajectory] = Controller_NMPC_CAN_v2(X, X_target, command, max_torque, idle, dt, params, XF)
    
    % MPC 매개변수
    Np = 1000;  % 예측 지평선
    Nc = 1000;   % 제어 지평선

    PosQ = 1;
    VelQ = 1;
    AttQ = 100000;
    RatQ = 100000;

    Qgain = diag([ PosQ PosQ VelQ VelQ AttQ RatQ]);
    Rgain = diag([ 1 1 1 1 1 1 1 1 1]);
    % Rgain = 1;



    % Qgain = 1;


    % 상태 공간 모델 매개변수
    I_Z = params.Airbearing.I_Z;   % [kgmm^2]
    d = params.Airbearing.D_ref;   % [mm] (CG to Thruster)
    m_A = params.Airbearing.m;       % [kg] (Airbearing mass)
    T = params.Airbearing.Thrust;  % [N] (Mean Thrust Force)
    
    % torque = max_torque * ((command/idle)-1);
    % torque = max_torque * (command - idle) / idle;

    

    % % 현재 상태
    % psi = X(5);
    % u0 = X(3);
    % v0 = X(4);
    % r0 = X(6);

    % 연속 시간 상태 공간 모델
    % Ac = [0 0 cos(psi) -sin(psi) -u0*sin(psi)-v0*cos(psi) 0;
    %       0 0 sin(psi)  cos(psi)  u0*cos(psi)-v0*sin(psi) 0;
    %       0 0    0       -r0             0               -v0;
    %       0 0   r0         0             0                u0;
    %       0 0    0         0             0                1;
    %       0 0    0         0             0                0];

     A = [
        0,  0,    cos(X(5)),   -sin(X(5)),  -X(3)*sin(X(5))-X(4)*cos(X(5)),      0;
        0,  0,    sin(X(5)),    cos(X(5)),   X(3)*cos(X(5))-X(4)*sin(X(5)),      0;
        0,  0,            0,        -X(6),                               0,  -X(4);
        0,  0,         X(6),            0,                               0,   X(3);
        0,  0,            0,            0,                               0,      1;
        0,  0,            0,            0,                               0,      0];


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
        -T*d/I_Z   T*d/I_Z   -T*d/I_Z    T*d/I_Z   -T*d/I_Z    T*d/I_Z   -T*d/I_Z    T*d/I_Z     max_torque/49/I_Z];

    % max_torque/(idle)
    % ((max_torque/idle)-1)/I_Z

    Cc = eye(6);

    % Dc = zeros(length(Cc(:, 1)), length(Bc(1, :)));

    tic;

    % 이산화
    % [Ad, Bd, Cd, Dd] = c2dm(Ac, Bc, Cc, Dc, dt);

    % Ad = [
    %         1, 0, dt*cos(X(5)), -dt*sin(X(5)),   dt * ( -X(3)*sin(X(5)) - X(4)*cos(X(5)) ),           0;
    %         0, 1, dt*sin(X(5)),  dt*cos(X(5)),   dt * (  X(3)*cos(X(5)) - X(4)*sin(X(5)) ),           0;
    %         0, 0,            1,      -dt*X(6),                                           0,  -dt * X(4);
    %         0, 0,      dt*X(6),             1,                                           0,   dt * X(3);
    %         0, 0,            0,             0,                                           1,          dt;
    %         0, 0,            0,             0,                                           0,           1
    %      ];

    % Ad = [
    %         1, 0, cos(X(5))*dt - X(6)*sin(X(5))*dt^2/2, -sin(X(5))*dt - X(6)*cos(X(5))*dt^2/2, (-X(3)*sin(X(5)) - X(4)*cos(X(5)))*dt + (-X(3)*X(6)*cos(X(5)) + X(4)*X(6)*sin(X(5)))*dt^2/2, (-X(3)*cos(X(5)) - X(4)*sin(X(5)))*dt^2/2;
    %         0, 1, sin(X(5))*dt + X(6)*cos(X(5))*dt^2/2, cos(X(5))*dt - X(6)*sin(X(5))*dt^2/2, (X(3)*cos(X(5)) - X(4)*sin(X(5)))*dt + (-X(3)*X(6)*sin(X(5)) - X(4)*X(6)*cos(X(5)))*dt^2/2, (-X(3)*sin(X(5)) + X(4)*cos(X(5)))*dt^2/2;
    %         0, 0, 1 - X(6)^2*dt^2/2, X(6)*dt, 0, X(4)*dt;
    %         0, 0, -X(6)*dt - X(6)^2*dt^2/2, 1 - X(6)^2*dt^2/2, 0, -X(3)*dt - X(6)*X(4)*dt^2/2;
    %         0, 0, 0, 0, 1, dt;
    %         0, 0, 0, 0, 0, 1
    %         ];


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

    for i = 1 : Np

        W_block = W_1 * ( A_a^( i - 1 ) );
        W = [ W; W_block ];

    end

    % Z 행렬 생성
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

    f = Z'*Q*(W*XF-Rr);

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
    
    % Y
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

    % all con
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

    % Motor_cmd = U( 9 );
    % 
    % % Scale the delta_w_rw to motor command (0-100, with 40 as idle)
    % min_input = -0.001; % Define minimum input based on your application
    % max_input = 0.001; % Define maximum input based on your application
    % 
    % motor_command = scale_command( Motor_cmd, min_input, max_input, 49 );
    % % Motor_mask = LIMIT2( motor_command, 0, 98 );
    % U( 9 ) = LIMIT2( motor_command, 0, 98 );

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