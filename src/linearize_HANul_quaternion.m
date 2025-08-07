function [A, B] = linearize_HANul_quaternion(X, U, params, dt)
    % linearize_HANul_quaternion: HANul 로켓의 쿼터니언 자세 동역학 선형화
    % 수치적 방법을 이용한 개선된 버전
    %
    % 입력:
    % X: 현재 상태 벡터 [pos; V_B; att_quat; omega] (13x1)
    % U: 현재 제어 입력 [카나드1-4, RCS1-4] (8x1)
    % params: 로켓 파라미터 구조체
    % dt: 샘플링 시간 [s]
    %
    % 출력:
    % A: 선형화된 상태 행렬 (7x7) [q0,q1,q2,q3,p,q,r]
    % B: 선형화된 입력 행렬 (7x8)
    
    % 디버깅: 함수 시작 표시
    persistent call_count
    if isempty(call_count)
        call_count = 0;
    end
    call_count = call_count + 1;
    
    if call_count <= 3
        disp(['linearize_HANul_quaternion 호출 #', num2str(call_count)]);
        disp(['상태 벡터 크기: ', num2str(size(X,1)), 'x', num2str(size(X,2))]);
        disp(['제어 입력 크기: ', num2str(size(U,1)), 'x', num2str(size(U,2))]);
    end
    
    % MPC 관련 상태/입력 부분만 추출
    quat = X(7:10);       % 쿼터니언 (q0,q1,q2,q3)
    omega = X(11:13);     % 각속도 (p,q,r)
    
    % 쿼터니언 정규화 (단위 크기 보장)
    quat = quat / norm(quat);
    
    % 수치적 선형화를 위한 미소 변화량 설정
    eps_quat = 1e-5;      % 쿼터니언 변화량 (매우 작게 설정)
    eps_omega = 1e-4;     % 각속도 변화량 [rad/s]
    eps_input = 1e-4;     % 제어 입력 변화량 (카나드 각도 및 RCS)
    
    % 프로세스 노이즈 (동역학 적분에 사용)
    Qf = zeros(6, 1);
    
    % 현재 시뮬레이션 시간 (vehicle_dynamics_HANul 호출에 필요)
    current_time = 0;  % 필요하다면 외부에서 전달받도록 수정 가능
    
    % A 행렬 계산 (수치적 방법)
    A = zeros(7, 7);
    
    % 주어진 상태에서 기준 미분값 계산
    % 수정: 마지막 dt 매개변수 제거
    [Xdot_nominal, ~, ~, ~, ~, ~, ~] = vehicle_dynamics_HANul(X, U, Qf, dt, params, current_time);
    xdot_nominal = [Xdot_nominal(7:10); Xdot_nominal(11:13)];  % 관심 있는 상태 부분만 추출
    
    % 각 상태 변수에 대한 자코비안 계산
    for i = 1:7
        % 변화된 상태 벡터 생성
        X_perturbed = X;
        
        if i <= 4  % 쿼터니언 요소 변화
            X_perturbed(6+i) = X(6+i) + eps_quat;
            % 쿼터니언 정규화
            X_perturbed(7:10) = X_perturbed(7:10) / norm(X_perturbed(7:10));
        else  % 각속도 요소 변화
            X_perturbed(10+i-4) = X(10+i-4) + eps_omega;
        end
        
        % 변화된 상태에서 미분값 계산
        % 수정: 마지막 dt 매개변수 제거
        [Xdot_perturbed, ~, ~, ~, ~, ~, ~] = vehicle_dynamics_HANul(X_perturbed, U, Qf, dt, params, current_time);
        xdot_perturbed = [Xdot_perturbed(7:10); Xdot_perturbed(11:13)];
        
        % 변화된 상태에 대한 미분값 - 명목 상태에 대한 미분값을 변화량으로 나눔
        if i <= 4
            delta_x = X_perturbed(6+i) - X(6+i);  % 실제 쿼터니언 변화량 계산
            A(:, i) = (xdot_perturbed - xdot_nominal) / delta_x;
        else
            A(:, i) = (xdot_perturbed - xdot_nominal) / eps_omega;
        end
    end
    
    % B 행렬 계산 (수치적 방법)
    B = zeros(7, 8);
    
    % 각 제어 입력에 대한 자코비안 계산
    for i = 1:8
        % 변화된 제어 입력 벡터 생성
        U_perturbed = U;
        
        if i <= 4  % 카나드 각도 변화
            U_perturbed(i) = U(i) + eps_input;
        else  % RCS 상태 변화 (이진 입력이지만 연속적으로 선형화)
            U_perturbed(i) = min(1, U(i) + eps_input);  % 0-1 범위 유지
        end
        
        % 변화된 제어 입력에서 미분값 계산
        % 수정: 마지막 dt 매개변수 제거
        [Xdot_perturbed, ~, ~, ~, ~, ~, ~] = vehicle_dynamics_HANul(X, U_perturbed, Qf, dt, params, current_time);
        xdot_perturbed = [Xdot_perturbed(7:10); Xdot_perturbed(11:13)];
        
        % 제어 입력 변화에 대한 미분값
        if i <= 4
            B(:, i) = (xdot_perturbed - xdot_nominal) / eps_input;
        else
            delta_u = U_perturbed(i) - U(i);  % 실제 RCS 변화량 계산
            if delta_u > 0
                B(:, i) = (xdot_perturbed - xdot_nominal) / delta_u;
            else
                B(:, i) = zeros(7, 1);  % 변화가 없으면 영향도 없음
            end
        end
    end
    
    % 상태 및 입력 행렬 검증 및 안정화
    % 소스 검사 및 불안정한 값 제거
    A(abs(A) < 1e-10) = 0;
    B(abs(B) < 1e-10) = 0;
    
    % 행렬 지수 방법 (더 정확한 이산화)
    A_d = expm(A * dt);
    
    % 이산 입력 행렬 계산
    % 연속 시간 행렬의 역행렬 존재 확인
    if rcond(A) > 1e-10
        % A가 특이행렬이 아닌 경우
        B_d = (A_d - eye(7)) * inv(A) * B;
    else
        % A가 특이행렬인 경우 대안적 방법 사용
        B_d = dt * B;
        
        % Zero-order hold 근사
        n = 10;  % 분할 횟수
        dt_sub = dt / n;
        A_sub = eye(7) + A * dt_sub;
        
        B_d = zeros(7, 8);
        A_temp = eye(7);
        
        for k = 1:n
            B_d = B_d + A_temp * B * dt_sub;
            A_temp = A_temp * A_sub;
        end
    end
    
    % NaN 체크 및 처리
    if any(isnan(A_d(:))) || any(isnan(B_d(:)))
        warning('선형화 행렬에 NaN 값이 있습니다. 안정화 수행...');
        
        % NaN 값 제거
        A_d(isnan(A_d)) = 0;
        B_d(isnan(B_d)) = 0;
        
        % 간단한 이산화로 대체
        if all(isnan(A_d(:)))
            A_d = eye(7) + A * dt;
        end
        
        if all(isnan(B_d(:)))
            B_d = B * dt;
        end
    end
    
    % % 행렬 조건 검사
    % A_cond = cond(A_d);
    % if A_cond > 1e10
    %     warning('A 행렬 조건수가 높습니다: %e. 행렬 안정화 수행...', A_cond);
    % 
    %     % 대각선 요소에 작은 값 추가하여 안정화
    %     A_d = A_d + 1e-10 * eye(7);
    % end
    % 
    % % 쿼터니언 역학 안정화 (쿼터니언 정규화 속성 반영)
    % % 쿼터니언 부분 행렬의 특이값 분해
    % [U_svd, S_svd, V_svd] = svd(A_d(1:4, 1:4));
    % 
    % % 특이값이 너무 크면 조정
    % max_singular = 0.99;  % 최대 특이값 제한
    % for i = 1:4
    %     if S_svd(i,i) > max_singular
    %         S_svd(i,i) = max_singular;
    %     end
    % end
    % 
    % % 조정된 쿼터니언 역학 행렬
    % A_d(1:4, 1:4) = U_svd * S_svd * V_svd';
    % 
    % % 디버깅: A, B 행렬 출력 (처음 몇 번만)
    % if call_count <= 2
    %     disp('--- A 행렬 (연속 시간) ---');
    %     disp(A);
    %     disp('--- B 행렬 (연속 시간) ---');
    %     disp(B);
    %     disp('--- A 행렬 (이산 시간) ---');
    %     disp(A_d);
    % end
    
    % 최종 결과 반환
    A = A_d;
    B = B_d;
end