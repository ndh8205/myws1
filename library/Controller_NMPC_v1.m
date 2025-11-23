function [U, computation_time, predicted_trajectory] = Controller_NMPC_v1(X, X_target, command, max_torque, idle, dt, params, XF)
     % Controller_NMPC_KJC_Nonterminal_v2: NMPC 제어기 함수
    %
    % 입력:
    %   X           - 현재 상태 벡터 (6x1)
    %   X_target    - 목표 상태 벡터 (6x1)
    %   command     - 명령 신호
    %   max_torque  - 최대 토크 제한
    %   idle        - 대기 상태
    %   dt          - 샘플링 시간
    %   params      - 시스템 파라미터
    %   XF          - 최종 상태 (6x1)
    %
    % 출력:
    %   U                     - 제어 입력 벡터 (9x1)
    %   computation_time      - 계산 시간
    %   predicted_trajectory  - 예측 궤적 (6x51)

    % 타이머 시작
    tic;

    %% MPC 매개변수 설정
    Np = 50;  % 예측 지평선
    Nc = 50;  % 제어 지평선

    % 가중치 설정
    PosQ = 0.1;
    VelQ = 1;
    AttQ = 600;
    RatQ = 2000;
    Qgain = diag([PosQ PosQ VelQ VelQ AttQ RatQ]);
    Rgain = diag([1 1 1 1 1 1 1 1 1]);

    %% 상태 공간 모델 매개변수
    I_Z = params.Airbearing.I_Z;     % [kgmm^2]
    d = params.Airbearing.D_ref;     % [mm]
    m_A = params.Airbearing.m;       % [kg]
    T = params.Airbearing.Thrust;    % [N]

    %% 연속 시간 상태 공간 모델
    Ac = [
        0,  0,    cos(X(5)),   -sin(X(5)),   0,      0;
        0,  0,    sin(X(5)),    cos(X(5)),   0,      0;
        0,  0,            0,        -X(6),      0,      0;
        0,  0,         X(6),            0,      0,      0;
        0,  0,            0,            0,      0,      1;
        0,  0,            0,            0,      0,      0 
    ];

    Bc = [
        0,            0,            0,           0,            0,           0,            0,           0,        0;
        0,            0,            0,           0,            0,           0,            0,           0,        0;
        0,       -T/m_A,       -T/m_A,           0,            0,       T/m_A,        T/m_A,           0,        0;
       -T/m_A,            0,            0,       T/m_A,        T/m_A,           0,            0,      -T/m_A,        0;
        0,            0,            0,           0,            0,           0,            0,           0,        0;
   -(d*T)/I_Z,    (d*T)/I_Z,   -(d*T)/I_Z,   (d*T)/I_Z,   -(d*T)/I_Z,   (d*T)/I_Z,   -(d*T)/I_Z,   (d*T)/I_Z,    1/I_Z
    ];

    Cc = eye(6);
    Dc = zeros(size(Cc, 1), size(Bc, 2));

    %% 이산화 (c2d 사용)
    sys_c = ss(Ac, Bc, Cc, Dc);
    sys_d = c2d(sys_c, dt);
    Ad = sys_d.A;
    Bd = sys_d.B;
    Cd = sys_d.C;
    Dd = sys_d.D;

    %% 가중치 행렬
    Q = kron(eye(Np), Qgain);  % 상태 추적 오차에 대한 가중치 (300x300)
    R = kron(eye(Nc), Rgain);  % 제어 변화에 대한 가중치 (450x450)

    %% 증강된 상태 공간 모델
    [n, ~] = size(Ad); % n = 6
    [~, m] = size(Bd); % m = 9
    [p, ~] = size(Cd); % p = 6

    % A_a를 정사각 행렬로 정의
    A_a = [Ad, zeros(n, p); Cd * Ad, eye(p)]; % (12x12)

    % B_a는 증강된 입력 행렬
    B_a = [Bd; Cd * Bd]; % (12x9)

    % C_a는 증강된 출력 행렬
    C_a = [zeros(p, n), eye(p)]; % (6x12)

    %% W 행렬 생성 (반복문 사용)
    W = zeros(p*Np, n + p);  % 300x12 사전 할당
    for i = 1:Np
        W((i-1)*p + 1:i*p, :) = C_a * (A_a)^(i-1);
    end

    %% Z 행렬 생성
    Z = zeros(p*Np, m*Nc); % 300x450 사전 할당
    for i = 1:Np
        for j = 1:Nc
            if i >= j
                Z((i-1)*p + 1:i*p, (j-1)*m + 1:j*m) = C_a * (A_a)^(i-j) * B_a;
            else
                Z((i-1)*p + 1:i*p, (j-1)*m + 1:j*m) = zeros(p, m);
            end
        end
    end

    %% 증강 상태 벡터 정의
    x_aug = [X; XF]; % 12x1 벡터

    %% 목표 상태 벡터 확장
    Y_ref = kron(ones(Np,1), X_target); % 300x1 벡터

    %% 최적화 비용 함수 구성
    f = sparse(Z' * Q * (W * x_aug - Y_ref)); % 450x1 벡터

    %% 제약 조건 설정
    % 제어 입력 제약 (예: 토크 제한)
    u_min = -max_torque / 6 * ones(m, 1); % 9x1
    u_max = max_torque / 6 * ones(m, 1);  % 9x1

    % 상태 제약 (예: 특정 상태 변수 제한)
    y_min = [-inf; -inf; -40; -40; -inf; -inf];
    y_max = [inf; inf; 40; 40; inf; inf];

    % U 제약
    U_min = kron(ones(Nc,1), u_min); % 450x1
    U_max = kron(ones(Nc,1), u_max); % 450x1

    % Y 제약
    Y_min = kron(ones(Np,1), y_min) - W * x_aug; % 300x1
    Y_max = kron(ones(Np,1), y_max) - W * x_aug; % 300x1

    % A 행렬과 b 행렬 구성
    A_ineq = [Z; -Z; eye(m*Nc); -eye(m*Nc)]; % (450+450+450+450)x450 = 1800x450
    b_ineq = [Y_max; -Y_min; U_max; -U_min]; % 1800x1

    %% Hessian 및 기울기 벡터 계산 (희소 행렬 사용)
    H = sparse(Z' * Q * Z + R); % 450x450
    % f는 이미 위에서 정의됨

    %% 최적화 문제 해결
    options = optimoptions('quadprog', 'Algorithm', 'interior-point-convex', 'Display', 'off');
    [DeltaU, ~, exitflag] = quadprog(H, f, A_ineq, b_ineq, [], [], [], [], [], options);

    % 최적화 실패 시 예외 처리
    if exitflag ~= 1
        warning('quadprog 최적화가 실패했습니다. 이전 제어 입력을 유지합니다.');
        DeltaU = zeros(m*Nc, 1); % 450x1
    end

    %% 최적 제어 입력 추출
    persistent u_prev

    if isempty(u_prev)
        u_prev = zeros(m, 1); % 9x1
    end

    deltau = DeltaU(1:m); % 9x1
    U = u_prev + deltau;    % 9x1

    %% 릴레이 제어 (ON-OFF)
    U(1:8) = double(U(1:8) >= mean(U(1:8)));

    %% 다음 단계를 위해 제어 입력 저장
    u_prev = U;

    %% 계산 시간 측정
    computation_time = toc;
    disp(['Computation time: ', num2str(computation_time), ' seconds']);

    %% 예측 궤적 계산
    predicted_trajectory = zeros(n, Np+1); % 6x51
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
