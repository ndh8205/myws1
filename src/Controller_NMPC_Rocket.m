function [U, computation_time, predicted_trajectory] = Controller_NMPC_Rocket(X, X_target, U_prev, dt, params)
    % Controller_NMPC_Rocket: 로켓 자세 제어용 비선형 MPC 제어기
    %
    % 입력:
    %   X: 현재 상태 벡터 [pos; V_B; att_quat; omega] (13x1)
    %   X_target: 목표 상태 벡터 (쿼터니언 형태의 목표 자세) (13x1)
    %   U_prev: 이전 제어 입력 (8x1)
    %   dt: 샘플링 시간 [s]
    %   params: 로켓 파라미터 구조체
    %
    % 출력:
    %   U: 최적 제어 입력 [카나드1-4, RCS1-4] (8x1)
    %   computation_time: 계산 시간 [s]
    %   predicted_trajectory: 예측된 상태 궤적 (13xNp)
    
    % MPC 매개변수 설정
    tic;
    
    Np = 20;  % 예측 지평선 (prediction horizon)
    Nc = 10;  % 제어 지평선 (control horizon)
    
    % 상태 추출
    pos = X(1:3);      % 위치 [m]
    V_B = X(4:6);      % 속도 [m/s]
    att_quat = X(7:10); % 쿼터니언 자세
    omega = X(11:13);  % 각속도 [rad/s]
    
    % 목표 상태 추출
    quat_target = X_target(7:10);
    
    % 자세 오차 계산 (쿼터니언 오차)
    quat_now = att_quat / norm(att_quat);  % 정규화
    quat_target = quat_target / norm(quat_target);  % 정규화
    inv_quat_state = inv_q(quat_now);
    quat_error = q2q_mult(inv_quat_state, quat_target);
    quat_error = quat_error / norm(quat_error);
    
    % 자세 오차 크기에 따른 가중치 조정
    angle_error = 2 * acos(abs(quat_error(1)));
    
    % 동적 가중치 조정 (오차가 크면 자세 제어에 높은 가중치)
    if angle_error > deg2rad(10)
        AttQ = 1000;   % 큰 자세 오차: 높은 가중치
        RateQ = 100;
    else
        AttQ = 10000;  % 작은 자세 오차: 매우 높은 가중치 (정밀 제어)
        RateQ = 1000;
    end
    
    % 가중치 행렬 설정
    Q_att = diag([AttQ, AttQ, AttQ]);         % 자세 오차 가중치
    Q_rate = diag([RateQ, RateQ, RateQ]);     % 각속도 오차 가중치
    R_canard = eye(4) * 0.1;                 % 카나드 제어 비용
    R_rcs = eye(4) * 1;                      % RCS 제어 비용
    
    % 상태 공간 모델 매개변수 추출
    J = params.vehicle.J;                    % 관성 텐서 [kg*m^2]
    Lt = params.vehicle.Lt;                  % 추력 중심-CG 거리 [m]
    Lc = params.vehicle.Lc;                  % 카나드-CG 거리 [m]
    Lrcs = params.vehicle.Lrcs;              % RCS-CG 거리 [m]
    m = params.vehicle.m_W;                 % 로켓 질량 [kg]
    
    % 현재 자세 쿼터니언에서 DCM 행렬 계산
    R_B2I = GetDCM_QUAT(quat_now);
    
    % 선형화된 자세 동역학 (각속도 -> 쿼터니언 변화율)
    omega_mat = [0, -omega(1), -omega(2), -omega(3);
                 omega(1), 0, omega(3), -omega(2);
                 omega(2), -omega(3), 0, omega(1);
                 omega(3), omega(2), -omega(1), 0];
    
    % 자세 표현을 단순화하기 위해 오일러 각으로 변환
    euler = Quat2Euler(quat_now');
    roll = euler(1);
    pitch = euler(2);
    yaw = euler(3);
    
    % 근사적 선형 자세 동역학 (오일러 각 기반)
    A_att = [0, 0, 0;
             0, 0, 0;
             0, 0, 0];
    
    B_att = [1, sin(roll)*tan(pitch), cos(roll)*tan(pitch);
             0, cos(roll), -sin(roll);
             0, sin(roll)/cos(pitch), cos(roll)/cos(pitch)];
    
    % 각속도 동역학 (모멘트 -> 각가속도)
    A_rate = zeros(3);
    B_rate = inv(J);
    
    % 제어 입력 매핑 행렬 (카나드 및 RCS -> 모멘트)
    % 각 제어기의 영향을 모델링
    B_control = zeros(3, 8);
    
    % 카나드 효과 계수 (공력 모멘트)
    eps1 = 1.0;  % Z축 모멘트에 대한 카나드 1의 효과
    eps2 = 1.0;  % Y축 모멘트에 대한 카나드 2의 효과
    eps3 = 1.0;  % Z축 모멘트에 대한 카나드 3의 효과
    eps4 = 1.0;  % Y축 모멘트에 대한 카나드 4의 효과
    
    % 카나드 효과 (1-4번)
    B_control(1, 1:4) = [0, 0, 0, 0];  % 롤에 영향 없음
    B_control(2, 1:4) = [0, eps2, 0, -eps4] * Lc;  % 피치 모멘트
    B_control(3, 1:4) = [eps1, 0, -eps3, 0] * Lc;  % 요 모멘트
    
    % RCS 추력기 효과 (5-8번)
    d = params.vehicle.r_ref;  % 로켓 반경 [m]
    T_RCS = params.vehicle.RCS_T;  % RCS 추력 [N]
    
    % RCS 추력기 효과 - 롤 제어용 (교차 배치 가정)
    B_control(1, 5:8) = [T_RCS*d, -T_RCS*d, -T_RCS*d, T_RCS*d];
    B_control(2, 5:8) = [0, 0, 0, 0];  % 피치에 영향 없음 (카나드가 담당)
    B_control(3, 5:8) = [0, 0, 0, 0];  % 요에 영향 없음 (카나드가 담당)
    
    % 최종 선형화된 상태 공간 모델 (간소화된 모델)
    % 상태: [롤, 피치, 요, p, q, r]
    % 제어: [카나드1-4, RCS1-4]
    Ac = [A_att, B_att;
          zeros(3), A_rate];
    Bc = [zeros(3, 8);
          B_rate * B_control];
    
    % 출력 행렬 (상태 관측)
    Cc = eye(6);
    Dc = zeros(6, 8);
    
    % 이산화
    [Ad, Bd, Cd, Dd] = c2dm(Ac, Bc, Cc, Dc, dt);
    
    % 가중치 행렬 구성
    Q = blkdiag(Q_att, Q_rate);
    R = blkdiag(R_canard, R_rcs);
    
    % 증강 상태 공간 모델 구성
    [n, ~] = size(Ad);
    [~, m] = size(Bd);
    [p, ~] = size(Cd);
    
    A_a = eye(n + p, n + p);
    A_a(1:n, 1:n) = Ad;
    A_a(n+1:n+p, 1:n) = Cd * Ad;
    
    B_a = zeros(n + p, m);
    B_a(1:n, :) = Bd;
    B_a(n+1:n+p, :) = Cd * Bd;
    
    C_a = zeros(p, n + p);
    C_a(:, n+1:n+p) = eye(p, p);
    
    % 예측 행렬 구성
    W = [];
    W_1 = C_a * A_a;
    
    for i = 1:Np
        W_block = W_1 * (A_a^(i-1));
        W = [W; W_block];
    end
    
    % 제어 행렬 구성
    Z = zeros(p * Np, m * Nc);
    
    for i = 1:Np
        for j = 1:Nc
            if i >= j
                Z((i-1)*p+1:i*p, (j-1)*m+1:j*m) = C_a * A_a^(i-j) * B_a;
            else
                Z((i-1)*p+1:i*p, (j-1)*m+1:j*m) = zeros(p, m);
            end
        end
    end
    
    % 가중치 및 목표 행렬 구성
    Q_mpc = [];
    R_mpc = [];
    Rr = [];
    
    % 현재 자세에서 오일러 각 추출
    euler_current = Quat2Euler(quat_now');
    
    % 목표 자세에서 오일러 각 추출
    euler_target = Quat2Euler(quat_target');
    
    % 각속도 목표 (안정상태에서는 0)
    omega_target = [0; 0; 0];
    
    % 오일러 각과 각속도 기반 목표 벡터
    target_vector = [euler_target; omega_target];
    
    for i = 1:Np
        Q_mpc = blkdiag(Q_mpc, Q);
        Rr = [Rr; target_vector];
    end
    
    for i = 1:Nc
        R_mpc = blkdiag(R_mpc, R);
    end
    
    % 현재 상태 벡터 구성 (오일러 각 + 각속도)
    x0 = [euler_current; omega];
    
    % 증강 상태 벡터
    x_a = [x0; Cd * x0];
    
    % Hessian 및 기울기 벡터 계산
    H = Z' * Q_mpc * Z + R_mpc;
    f = Z' * Q_mpc * (W * x_a - Rr);
    
    % 제어 입력 제약 조건
    % 카나드 각도 제한
    canard_max = deg2rad(20);  % 최대 각도 [rad]
    canard_min = deg2rad(-20); % 최소 각도 [rad]
    
    % RCS 추력기는 ON/OFF (0 또는 1)
    rcs_min = 0;
    rcs_max = 1;
    
    % 제약 조건 벡터
    u_min = [canard_min; canard_min; canard_min; canard_min; rcs_min; rcs_min; rcs_min; rcs_min];
    u_max = [canard_max; canard_max; canard_max; canard_max; rcs_max; rcs_max; rcs_max; rcs_max];
    
    % 전체 제어 지평선에 대한 제약 조건
    U_min = repmat(u_min, Nc, 1);
    U_max = repmat(u_max, Nc, 1);
    
    % 선형 제약 조건 행렬
    A_ineq = [eye(m*Nc); -eye(m*Nc)];
    b_ineq = [U_max; -U_min];
    
    % 최적화 문제 해결
    options = optimset('Algorithm', 'interior-point-convex', 'Display', 'off');
    try
        [DeltaU, ~, exitflag] = quadprog(H, f, A_ineq, b_ineq, [], [], [], [], [], options);
        
        if exitflag < 0
            warning('MPC 최적화 실패: exitflag = %d', exitflag);
            % 이전 제어 입력 유지 (기본값)
            U = U_prev;
        else
            % 첫 번째 제어 입력만 추출
            U = DeltaU(1:m);
            
            % 카나드 각도 제한 적용
            for i = 1:4
                U(i) = max(canard_min, min(canard_max, U(i)));
            end
            
            % RCS 추력기 이진화 (ON/OFF)
            for i = 5:8
                U(i) = round(U(i));  % 0 또는 1로 반올림
            end
        end
    catch ME
        warning('MPC 최적화 오류: %s', E.message);
        U = U_prev;  % 오류 발생 시 이전 제어 입력 유지
    end
    
    % 계산 시간 측정
    computation_time = toc;
    
    % 예측 궤적 계산
    predicted_trajectory = zeros(6, Np+1);
    predicted_trajectory(:,1) = x0;
    
    u_pred = U;
    for i = 1:Np
        predicted_trajectory(:,i+1) = Ad * predicted_trajectory(:,i) + Bd * u_pred;
    end
end

% 쿼터니언 역수 계산 함수
function q_inv = inv_q(q)
    q_inv = [q(1); -q(2:4)];
    q_inv = q_inv / norm(q_inv);
end

% 쿼터니언 곱셈 함수
function q_res = q2q_mult(q1, q2)
    q_res = zeros(4, 1);
    
    q_res(1) = q1(1)*q2(1) - q1(2)*q2(2) - q1(3)*q2(3) - q1(4)*q2(4);
    q_res(2) = q1(1)*q2(2) + q1(2)*q2(1) + q1(3)*q2(4) - q1(4)*q2(3);
    q_res(3) = q1(1)*q2(3) - q1(2)*q2(4) + q1(3)*q2(1) + q1(4)*q2(2);
    q_res(4) = q1(1)*q2(4) + q1(2)*q2(3) - q1(3)*q2(2) + q1(4)*q2(1);
    
    q_res = q_res / norm(q_res);
end