function [A, B] = linearize_HANul_attitude(X, U, params, dt)
    % linearize_HANul_attitude: HANul 로켓 자세 동역학 선형화
    %
    % 입력:
    %   X: 현재 상태 벡터 [pos; V_B; att_quat; omega] (13x1)
    %   U: 현재 제어 입력 [카나드1-4, RCS1-4] (8x1)
    %   params: 로켓 파라미터 구조체
    %   dt: 샘플링 시간 [s]
    %
    % 출력:
    %   A: 선형화된 상태 행렬 (6x6)
    %   B: 선형화된 입력 행렬 (6x8)
    
    % 상태 및 제어 입력 추출
    att_quat = X(7:10)';   % 쿼터니언 자세 (행 벡터로 변환)
    omega = X(11:13);     % 각속도 [rad/s]
    
    % 수치적 방법으로 선형화 수행
    % 중심 차분법(Central Difference Method) 사용
    
    % 초기 상태 백업
    X_orig = X;
    U_orig = U;
    
    % 작은 섭동 크기 설정
    h_state = 1e-6;  % 상태 변수 섭동
    h_input = 1e-6;  % 제어 입력 섭동
    
    % 상태 벡터를 오일러 각과 각속도로 변환
    euler = Quat2Euler(att_quat);
    X_euler = [euler; omega];  % [phi; theta; psi; p; q; r]
    
    % A 행렬 계산 (6x6)
    A = zeros(6, 6);
    
    % 각 상태 변수에 대한 미분 계산
    for i = 1:6
        % 상태 섭동 벡터 생성
        dX_euler = zeros(6, 1);
        dX_euler(i) = h_state;
        
        % 섭동된 오일러 각/각속도 계산
        X_euler_perturbed_plus = X_euler + dX_euler;
        X_euler_perturbed_minus = X_euler - dX_euler;
        
        % 오일러 각 섭동 (i=1,2,3)이면 쿼터니언으로 변환
        if i <= 3
            % 양의 섭동에 대한 쿼터니언 변환
            quat_plus = GetQUAT(X_euler_perturbed_plus(3), X_euler_perturbed_plus(2), X_euler_perturbed_plus(1));
            X_plus = X_orig;
            X_plus(7:10) = quat_plus';  % 열 벡터로 변환
            X_plus(11:13) = X_euler_perturbed_plus(4:6);
            
            % 음의 섭동에 대한 쿼터니언 변환
            quat_minus = GetQUAT(X_euler_perturbed_minus(3), X_euler_perturbed_minus(2), X_euler_perturbed_minus(1));
            X_minus = X_orig;
            X_minus(7:10) = quat_minus';  % 열 벡터로 변환
            X_minus(11:13) = X_euler_perturbed_minus(4:6);
        else
            % 각속도 섭동 (i=4,5,6)
            X_plus = X_orig;
            X_plus(11:13) = X_euler_perturbed_plus(4:6);
            
            X_minus = X_orig;
            X_minus(11:13) = X_euler_perturbed_minus(4:6);
        end
        
        % 각 섭동에 대한 미래 상태 계산 (현재 시간을 0으로 가정)
        [X_dot_plus, ~, ~, ~, ~, ~, ~] = vehicle_dynamics_HANul(X_plus, U_orig, zeros(6,1), dt, params, 0);
        [X_dot_minus, ~, ~, ~, ~, ~, ~] = vehicle_dynamics_HANul(X_minus, U_orig, zeros(6,1), dt, params, 0);
        
        % 쿼터니언 및 각속도 미분 추출
        quat_dot_plus = X_dot_plus(7:10);
        omega_dot_plus = X_dot_plus(11:13);
        
        quat_dot_minus = X_dot_minus(7:10);
        omega_dot_minus = X_dot_minus(11:13);
        
        % 오일러 각 미분 계산
        euler_dot_plus = euler_rates_from_quaternion(att_quat, quat_dot_plus, X_plus(11:13));
        euler_dot_minus = euler_rates_from_quaternion(att_quat, quat_dot_minus, X_minus(11:13));
        
        % 전체 자세 동역학 미분 모음
        X_euler_dot_plus = [euler_dot_plus; omega_dot_plus];
        X_euler_dot_minus = [euler_dot_minus; omega_dot_minus];
        
        % 중심 차분법으로 A 행렬 계산
        A(:, i) = (X_euler_dot_plus - X_euler_dot_minus) / (2 * h_state);
    end
    
    % B 행렬 계산 (6x8)
    B = zeros(6, 8);
    
    % 각 제어 입력에 대한 미분 계산
    for j = 1:8
        % 제어 입력 섭동 벡터 생성
        dU = zeros(8, 1);
        dU(j) = h_input;
        
        % 섭동된 제어 입력
        U_plus = U_orig + dU;
        U_minus = U_orig - dU;
        
        % 각 섭동에 대한 상태 미분 계산
        [X_dot_plus, ~, ~, ~, ~, ~, ~] = vehicle_dynamics_HANul(X_orig, U_plus, zeros(6,1), dt, params, 0);
        [X_dot_minus, ~, ~, ~, ~, ~, ~] = vehicle_dynamics_HANul(X_orig, U_minus, zeros(6,1), dt, params, 0);
        
        % 쿼터니언 및 각속도 미분 추출
        quat_dot_plus = X_dot_plus(7:10);
        omega_dot_plus = X_dot_plus(11:13);
        
        quat_dot_minus = X_dot_minus(7:10);
        omega_dot_minus = X_dot_minus(11:13);
        
        % 오일러 각 미분 계산
        euler_dot_plus = euler_rates_from_quaternion(att_quat, quat_dot_plus, omega);
        euler_dot_minus = euler_rates_from_quaternion(att_quat, quat_dot_minus, omega);
        
        % 전체 자세 동역학 미분 모음
        X_euler_dot_plus = [euler_dot_plus; omega_dot_plus];
        X_euler_dot_minus = [euler_dot_minus; omega_dot_minus];
        
        % 중심 차분법으로 B 행렬 계산
        B(:, j) = (X_euler_dot_plus - X_euler_dot_minus) / (2 * h_input);
    end
    
    % 연속 시간 모델을 이산 시간 모델로 변환
    [A, B] = c2d(A, B, dt);
end

function euler_rates = euler_rates_from_quaternion(q, qdot, omega)
    % 쿼터니언 미분으로부터 오일러 각 변화율 계산
    % q: 쿼터니언 (행 벡터)
    % qdot: 쿼터니언 미분 (열 벡터)
    % omega: 각속도 (열 벡터)
    
    % 현재 오일러 각 계산
    euler = Quat2Euler(q);
    phi = euler(1);    % 롤
    theta = euler(2);  % 피치
    
    % 특이점 방지 (pitch가 ±90도에 가까울 때)
    if abs(cos(theta)) < 1e-6
        theta = theta + sign(theta) * 1e-6;
    end
    
    % 각속도에서 오일러 각 변화율로의 변환 행렬
    H = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
         0, cos(phi), -sin(phi);
         0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
    
    % 오일러 각 변화율 계산
    euler_rates = H * omega;
end