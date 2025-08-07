function [U, computation_time, predicted_trajectory] = Controller_MPC_HANul(X, Command_Vector, U_prev, dt, params)
    % Controller_MPC_HANul: HANul 로켓용 자세 전용 MPC 제어기
    %
    % 입력:
    %   X: 현재 상태 벡터 [pos; V_B; att_quat; omega] (13x1)
    %   Command_Vector: 목표 자세 쿼터니언 (4x1)
    %   U_prev: 이전 제어 입력 [카나드1-4, RCS1-4] (8x1)
    %   dt: 샘플링 시간 [s]
    %   params: 로켓 파라미터 구조체
    %
    % 출력:
    %   U: 최적 제어 입력 [카나드1-4, RCS1-4] (8x1)
    %   computation_time: 계산 시간 [s]
    %   predicted_trajectory: 예측된 자세 궤적 (오일러 각 + 각속도)
    
    % 계산 시간 측정 시작
    tic;
    
    % MPC 매개변수 설정
    Np = 10;  % 예측 지평선 (prediction horizon)
    Nc = 5;   % 제어 지평선 (control horizon)
    
    % 상태 추출
    att_quat = X(7:10)';   % 쿼터니언 자세 (행 벡터로 변환)
    omega = X(11:13);     % 각속도 [rad/s]
    vel_body = X(4:6);    % 바디 프레임 속도 [m/s]
    
    % 목표 자세 추출
    % Command_Vector가 쿼터니언 형태로 전달되는 경우 처리
    % generate_commands_HANul 함수로부터 받은 쿼터니언을 사용
    quat_target = Command_Vector';  % 행 벡터로 변환
    
    % 쿼터니언을 오일러 각으로 변환
    euler = Quat2Euler(att_quat);
    
    % 목표 자세를 오일러 각으로 변환
    euler_target = Quat2Euler(quat_target);
    
    % 자세 오차 계산 (쿼터니언 기반)
    att_quat_col = att_quat';  % 열 벡터로 변환
    quat_target_col = quat_target';  % 열 벡터로 변환
    inv_quat_state = inv_q(att_quat_col);
    quat_error = q2q_mult(inv_quat_state, quat_target_col);
    
    % 동역학 조건 판단 (저속 영역, 고속 영역 구분)
    airspeed = norm(vel_body);
    
    % 자세 오차의 크기 계산 (오일러 각 기준)
    euler_error = euler_target - euler;
    error_magnitude = norm(euler_error);
    
    % params 구조체에서 로켓 파라미터 추출 (직접 활용)
    Lc = params.vehicle.Lc;        % CG에서 카나드까지의 거리 [m]
    Lrcs = params.vehicle.Lrcs;    % CG에서 RCS까지의 거리 [m]
    J_X = params.vehicle.J_X;      % X축 관성 모멘트
    J_Y = params.vehicle.J_Y;      % Y축 관성 모멘트
    J_Z = params.vehicle.J_Z;      % Z축 관성 모멘트
    D_ref = params.vehicle.D_ref;  % 로켓 직경 [m]
    RCS_T = params.vehicle.RCS_T;  % RCS 추력 [N]
    
    % 카나드 효과 계수 (params에서 직접 가져옴)
    eps1 = params.vehicle.canard_eps1;
    eps2 = params.vehicle.canard_eps2;
    eps3 = params.vehicle.canard_eps3;
    eps4 = params.vehicle.canard_eps4;
    
    % 현재 비행 상태에 따른 가중치 조정
    % 1. 공력 효과에 따른 조정 (속도 기반)
    if airspeed < 10.0  % 저속 영역 (10 m/s 미만)
        % 저속: RCS 위주 제어
        base_Q_att = 1000;    % 기본 자세 가중치
        base_Q_rate = 100;    % 기본 각속도 가중치
        R_canard = 10.0;      % 카나드 제어 비용 (높게 설정)
        R_rcs = 0.1;          % RCS 제어 비용 (낮게 설정)
    else  % 고속 영역 (10 m/s 이상)
        % 고속: 카나드 위주 제어
        base_Q_att = 1000;    % 기본 자세 가중치
        base_Q_rate = 100;    % 기본 각속도 가중치
        R_canard = 0.1;       % 카나드 제어 비용 (낮게 설정)
        R_rcs = 10.0;         % RCS 제어 비용 (높게 설정)
    end
    
    % 2. 자세 오차 크기에 따른 조정
    if error_magnitude > deg2rad(10)
        % 큰 오차: 응답 속도 중시
        Q_att_factor = 0.5;   % 자세 가중치 감소
        Q_rate_factor = 2.0;  % 각속도 가중치 증가
    else
        % 작은 오차: 정밀 제어 중시
        Q_att_factor = 2.0;   % 자세 가중치 증가
        Q_rate_factor = 0.5;  % 각속도 가중치 감소
    end
    
    % 3. 자세 축별 중요도 조정 (롤, 피치, 요)
    % 피치와 요의 제어를 롤보다 우선시
    Q_phi = base_Q_att * Q_att_factor * 0.5;  % 롤 가중치 (낮게 설정)
    Q_theta = base_Q_att * Q_att_factor * 1.0;  % 피치 가중치
    Q_psi = base_Q_att * Q_att_factor * 1.0;  % 요 가중치
    
    % 각속도 가중치 설정
    Q_p = base_Q_rate * Q_rate_factor;  % 롤 각속도 가중치
    Q_q = base_Q_rate * Q_rate_factor;  % 피치 각속도 가중치
    Q_r = base_Q_rate * Q_rate_factor;  % 요 각속도 가중치
    
    % 가중치 행렬 설정
    Q = diag([Q_phi, Q_theta, Q_psi, Q_p, Q_q, Q_r]);  % 상태 가중치
    R = diag([R_canard, R_canard, R_canard, R_canard, R_rcs, R_rcs, R_rcs, R_rcs]);  % 제어 가중치
    
    % 선형화된 상태 공간 모델 구하기
    [A, B] = linearize_HANul_attitude(X, U_prev, params, dt);
    
    % 현재 상태 벡터 (오일러 각 + 각속도)
    x_current = [euler; omega];
    
    % 목표 상태 벡터
    x_target = [euler_target; zeros(3, 1)];  % 목표 자세 + 0 각속도
    
    % MPC 예측 및 제어 행렬 구성
    [F, G] = mpc_prediction_matrices(A, B, Np, Nc);
    
    % 비용 함수 행렬 구성
    Q_bar = kron(eye(Np), Q);
    R_bar = kron(eye(Nc), R);
    
    % 목표 궤적 (전체 예측 지평선에 대해)
    T = repmat(x_target, Np, 1);
    
    % 현재 상태에서의 자유 응답
    X_free = F * x_current;
    
    % 2차 계획법 문제 설정
    H = G' * Q_bar * G + R_bar;
    f = 2 * G' * Q_bar * (X_free - T);
    
    % 제약 조건
    % 카나드 각도 제한 (파라미터에서 직접 가져옴)
    canard_max = params.vehicle.canard_max_angle;  % 최대 각도 [rad]
    canard_min = params.vehicle.canard_min_angle;  % 최소 각도 [rad]
    
    % RCS 추력기는 ON/OFF (0 또는 1)
    rcs_min = 0;
    rcs_max = 1;
    
    % 제약 행렬 구성
    % U 제약 (전체 제어 지평선에 대해)
    lb = repmat([canard_min; canard_min; canard_min; canard_min; rcs_min; rcs_min; rcs_min; rcs_min], Nc, 1);
    ub = repmat([canard_max; canard_max; canard_max; canard_max; rcs_max; rcs_max; rcs_max; rcs_max], Nc, 1);
    
    % 최적화 문제 해결
    options = optimoptions('quadprog', 'Algorithm', 'interior-point-convex', 'Display', 'off');
    
    try
        [U_sequence, ~, exitflag] = quadprog(H, f, [], [], [], [], lb, ub, [], options);
        
        if exitflag < 0
            warning('MPC 최적화 실패: exitflag = %d', exitflag);
            U = U_prev;  % 이전 제어 입력 유지
        else
            % 첫 번째 제어 입력 추출
            U = U_sequence(1:8);
            
            % RCS 추력기 ON/OFF 이진화
            for i = 5:8
                U(i) = round(U(i));  % 0 또는 1로 반올림
            end
        end
    catch ME
        warning('MPC 최적화 오류: %s', E.message);
        U = U_prev;  % 이전 제어 입력 유지
    end
    
    % 계산 시간 측정
    computation_time = toc;
    
    % 예측 궤적 계산
    predicted_trajectory = zeros(6, Np+1);
    predicted_trajectory(:,1) = x_current;
    
    % 제어 입력 시퀀스 재구성
    U_seq = zeros(8, Np);
    for i = 1:min(Nc, Np)
        U_seq(:,i) = U_sequence((i-1)*8+1:i*8);
    end
    
    if Nc < Np
        U_seq(:,Nc+1:end) = repmat(U_seq(:,Nc), 1, Np-Nc);
    end
    
    % 예측 궤적 시뮬레이션
    for i = 1:Np
        predicted_trajectory(:,i+1) = A * predicted_trajectory(:,i) + B * U_seq(:,i);
    end
end

function [F, G] = mpc_prediction_matrices(A, B, Np, Nc)
    % MPC 예측 행렬 구성
    % A: 시스템 행렬 (상태 전이)
    % B: 입력 행렬 (제어 효과)
    % Np: 예측 지평선
    % Nc: 제어 지평선
    
    [nx, ~] = size(A);  % 상태 차원
    [~, nu] = size(B);  % 제어 입력 차원
    
    % 자유 응답 행렬
    F = zeros(nx*Np, nx);
    for i = 1:Np
        F((i-1)*nx+1:i*nx, :) = A^i;
    end
    
    % 제어 응답 행렬
    G = zeros(nx*Np, nu*Nc);
    for i = 1:Np
        for j = 1:min(i, Nc)
            row_idx = (i-1)*nx+1:i*nx;
            col_idx = (j-1)*nu+1:j*nu;
            G(row_idx, col_idx) = A^(i-j) * B;
        end
    end
end