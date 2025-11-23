function [U, computation_time, predicted_trajectory] = Controller_NMPC_KJC(X, X_target, dt, params)
    % MPC 매개변수
    Np = 100;  % 예측 지평선
    Nc = 10;   % 제어 지평선


    % 상태 공간 모델 매개변수
    I_Z = params.Airbearing.I_Z;  % [kgmm^2]
    d = params.Airbearing.D_ref;      % [mm] (CG to Thruster)
    m = params.Airbearing.m;      % [kg] (Airbearing mass)
    T = params.Airbearing.Thrust;      % [N] (Mean Thrust Force)

    % 현재 상태
    psi = X(5);
    u0 = X(3);
    v0 = X(4);
    r0 = X(6);

    % 연속 시간 상태 공간 모델
    Ac = [0 0 cos(psi) -sin(psi) -u0*sin(psi)-v0*cos(psi) 0;
          0 0 sin(psi)  cos(psi)  u0*cos(psi)-v0*sin(psi) 0;
          0 0    0       -r0             0               -v0;
          0 0   r0         0             0                u0;
          0 0    0         0             0                1;
          0 0    0         0             0                0];

    Bc = [0      0      0      0      0      0      0      0;
          0      0      0      0      0      0      0      0;
          0    -T/m   -T/m     0      0     T/m    T/m     0;
        -T/m     0      0     T/m    T/m     0      0    -T/m;
          0      0      0      0      0      0      0      0;
        -T*d/I_Z   T*d/I_Z   -T*d/I_Z    T*d/I_Z   -T*d/I_Z    T*d/I_Z   -T*d/I_Z    T*d/I_Z];

    Cc = eye(6);
    Dc = zeros(length(Cc(:, 1)), length(Bc(1, :)));

    % 이산화
    [Ad, Bd, Cd, Dd] = c2dm(Ac, Bc, Cc, Dc, dt);

    % 가중치 행렬
    q = eye(size(Cd,1))*1000;  % 상태 추적 오차에 대한 가중치
    r = eye(size(Bd,2))*1;     % 제어 변화에 대한 가중치

    % 증강된 상태 공간 모델
    [n, ~] = size(Ad);
    [~, m] = size(Bd);
    [p, ~] = size(Cd);

    A_a = eye(n+p,n+p);
    A_a(1:n,1:n) = Ad;
    A_a(n+1:n+p,1:n) = Cd*Ad;

    B_a = zeros(n+p,m);
    B_a(1:n,:) = Bd;
    B_a(n+1:n+p,:) = Cd*Bd;

    C_a = zeros(p,n+p);
    C_a(:,n+1:n+p) = eye(p,p);

    % W 행렬 생성
    W = [];
    W_1 = C_a*A_a;
    for i = 1:Np
        W_block = W_1 * (A_a^(i-1));
        W = [W; W_block];
    end

    % Z 행렬 생성
    Z = zeros(p*Np,m*Nc);
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
        Q = blkdiag(Q,q);
        Rr = [Rr;X_target];
    end
    for i = 1:Nc
        R = blkdiag(R,r);
    end

    % Hessian 및 기울기 벡터 계산
    H = Z'*Q*Z + R;
    Nc_block = [eye(m,m),zeros(m,m*Nc-m)];

    % 현재 상태 업데이트
    persistent u_prev
    if isempty(u_prev)
        u_prev = zeros(m, 1);
    end
    x1 =  Ad*X + Bd*u_prev;
    y = Cd*x1;
    dx = x1 - X;
    XF = [dx; y];

    f = Z'*Q*(W*XF-Rr);

    % 제약 조건
    y_min = [-inf; -inf; -40; -40; -inf; -inf];
    y_max = [inf; inf; 40; 40; inf; inf];

    Y_min = repmat(y_min, Np, 1);
    Y_max = repmat(y_max, Np, 1);

    C2 = [-Z; Z];
    Acon = C2;
    C_min_max = [-Y_min + W*XF; Y_max - W*XF];

    % 최적화 문제 해결
    options = optimset('Algorithm', 'interior-point-convex', 'Display', 'off');
    tic;
    [DeltaU, ~, exitflag, ~] = quadprog(H, f, Acon, C_min_max, [], [], [], [], [], options);
    computation_time = toc;

    % 최적 제어 입력 추출
    deltau = Nc_block * DeltaU;
    U = u_prev + deltau;

    % 릴레이 제어 (ON-OFF)
    U = double(U >= mean(U));

    % 다음 단계를 위해 제어 입력 저장
    u_prev = U;

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