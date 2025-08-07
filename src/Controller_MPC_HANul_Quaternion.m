function [U, computation_time, predicted_trajectory] = Controller_MPC_HANul_Quaternion(X, Command_Vector, U_prev, dt, params)
    % Controller_MPC_HANul_Quaternion: HANul 로켓용 쿼터니언 기반 MPC 제어기
    %
    % 입력:
    %   X: 현재 상태 벡터 [pos; V_B; att_quat; omega] (13x1)
    %   Command_Vector: 목표 자세 쿼터니언 (1x4 또는 4x1)
    %   U_prev: 이전 제어 입력 [카나드1-4, RCS1-4] (8x1)
    %   dt: 샘플링 시간 [s]
    %   params: 로켓 파라미터 구조체
    %
    % 출력:
    %   U: 최적 제어 입력 [카나드1-4, RCS1-4] (8x1)
    %   computation_time: 계산 시간 [s]
    %   predicted_trajectory: 예측된 쿼터니언 자세 궤적 (쿼터니언 + 각속도)

    % 계산 시간 측정 시작
    tic;

    % MPC 매개변수 설정
    Np = 10;  % 예측 지평선 (prediction horizon)
    Nc = 5;   % 제어 지평선 (control horizon)

    % 상태 추출
    quat_current = X(7:10);   % 쿼터니언 자세 
    omega = X(11:13);         % 각속도 [rad/s]
    vel_body = X(4:6);        % 바디 프레임 속도 [m/s]

    % 목표 자세 쿼터니언 - 열벡터로 변환
    if size(Command_Vector, 1) == 1  % 행벡터인 경우
        quat_target = Command_Vector';  % 열벡터로 변환
    else
        quat_target = Command_Vector;   % 이미 열벡터
    end

    % 쿼터니언 정규화 (단위 크기 보장)
    quat_current = quat_current / norm(quat_current);
    quat_target = quat_target / norm(quat_target); 

    % 단순화된 가중치 설정
    % 상태 가중치 (쿼터니언 오차와 각속도에 대한 가중치)
    Q_quat = 10000;   
    Q_rate_roll = 85; 
    Q_rate_pitch = 1;
    Q_rate_yaw = 1;

    % 제어 입력 가중치 (카나드와 RCS 추력기에 대한 가중치)
    R_canard = 0.5;  % 카나드 제어 비용
    R_rcs = 10;    % RCS 추력기 제어 비용

    % 쿼터니언 및 각속도 가중치 행렬
    Q = diag([Q_quat, Q_quat, Q_quat, Q_quat, Q_rate_roll, Q_rate_pitch, Q_rate_yaw]);

    % 제어 입력 가중치 행렬
    R = diag([R_canard, R_canard, R_canard, R_canard, R_rcs, R_rcs, R_rcs, R_rcs]);

    try
        % 선형화된 상태 공간 모델 구하기
        disp('=== 선형화 모델 계산 시작 ===');
        [A, B] = linearize_HANul_quaternion(X, U_prev, params, dt);

        % 디버깅: A, B 행렬 크기 검증
        disp(['A 행렬 크기: ', num2str(size(A,1)), 'x', num2str(size(A,2))]);
        disp(['B 행렬 크기: ', num2str(size(B,1)), 'x', num2str(size(B,2))]);

        % 현재 상태 벡터 (쿼터니언 + 각속도)
        x_current = [quat_current; omega];

        % 목표 상태 벡터
        x_target = [quat_target; zeros(3, 1)];  % 목표 쿼터니언 + 0 각속도

        % 디버깅: 상태 벡터 크기 확인
        disp(['x_current 크기: ', num2str(size(x_current,1)), 'x', num2str(size(x_current,2))]);
        disp(['x_target 크기: ', num2str(size(x_target,1)), 'x', num2str(size(x_target,2))]);

        % MPC 예측 및 제어 행렬 구성
        disp('=== MPC 예측 행렬 계산 시작 ===');
        [F, G] = mpc_prediction_matrices(A, B, Np, Nc);

        % 디버깅: F, G 행렬 크기 검증
        disp(['F 행렬 크기: ', num2str(size(F,1)), 'x', num2str(size(F,2))]);
        disp(['G 행렬 크기: ', num2str(size(G,1)), 'x', num2str(size(G,2))]);

        % 목표 궤적 (전체 예측 지평선에 대해)
        T = repmat(x_target, Np, 1);

        % 현재 상태에서의 자유 응답
        X_free = F * x_current;

        % 디버깅: 목표 궤적과 자유 응답 크기 확인
        disp(['T 크기: ', num2str(size(T,1)), 'x', num2str(size(T,2))]);
        disp(['X_free 크기: ', num2str(size(X_free,1)), 'x', num2str(size(X_free,2))]);

        % 비용 함수 행렬 구성
        Q_bar = kron(eye(Np), Q);
        R_bar = kron(eye(Nc), R);

        % 디버깅: 비용 함수 행렬 크기 확인
        disp(['Q_bar 크기: ', num2str(size(Q_bar,1)), 'x', num2str(size(Q_bar,2))]);
        disp(['R_bar 크기: ', num2str(size(R_bar,1)), 'x', num2str(size(R_bar,2))]);

        % 2차 계획법 문제 설정
        H = G' * Q_bar * G + R_bar;
        f = 2 * G' * Q_bar * (X_free - T);

        % 디버깅: H, f 크기 확인
        disp(['H 크기: ', num2str(size(H,1)), 'x', num2str(size(H,2))]);
        disp(['f 크기: ', num2str(size(f,1)), 'x', num2str(size(f,2))]);

        % H와 f의 유효성 검사
        if any(isnan(H(:))) || any(isnan(f))
            warning('H 또는 f에 NaN 값이 있습니다!');
            disp(['H 행렬 NaN 개수: ', num2str(sum(isnan(H(:))))]);
            disp(['f 벡터 NaN 개수: ', num2str(sum(isnan(f)))]);

            % NaN 값 대체
            H(isnan(H)) = 0;
            f(isnan(f)) = 0;
            disp('NaN 값을 0으로 대체했습니다.');
        end

        % H 행렬 대칭성 확인 및 조정
        if ~isequal(H, H')
            disp('H 행렬이 대칭이 아닙니다. 대칭으로 조정합니다.');
            H = (H + H') / 2;
        end

        % H 행렬 조건수 확인
        try
            cond_H = cond(H);
            disp(['H 행렬 조건수: ', num2str(cond_H)]);

            if cond_H > 1e14
                warning('H 행렬 조건수가 매우 높습니다. 수치적 불안정성 가능성.');
                % 작은 값을 대각선에 추가하여 안정화
                disp('H 행렬 안정화 시도...');
                H = H + 1e-6 * eye(size(H,1));
            end
        catch ME
            warning('H 행렬 조건수 계산 실패: %s', ME.message);
        end

        % 제약 조건
        % 카나드 각도 제한
        canard_max = params.vehicle.canard_max_angle;  % 최대 각도 [rad]
        canard_min = params.vehicle.canard_min_angle;  % 최소 각도 [rad]

        % 매우 작은 값을 강제로 할당하여 제약 작동 확인
        if canard_max == 0
            canard_max = pi/6;  % 30도
            disp('canard_max가 0이므로 30도(pi/6)로 설정');
        end

        if canard_min == 0
            canard_min = -pi/6;  % -30도
            disp('canard_min이 0이므로 -30도(-pi/6)로 설정');
        end

        % RCS 추력기 제약
        rcs_min = 0;
        rcs_max = 3;

        % 제약 행렬 구성
        lb = repmat([canard_min; canard_min; canard_min; canard_min; rcs_min; rcs_min; rcs_min; rcs_min], Nc, 1);
        ub = repmat([canard_max; canard_max; canard_max; canard_max; rcs_max; rcs_max; rcs_max; rcs_max], Nc, 1);

        % 디버깅: 제약 조건 확인
        disp(['카나드 제약: [', num2str(canard_min), ', ', num2str(canard_max), '] rad']);
        disp(['RCS 제약: [', num2str(rcs_min), ', ', num2str(rcs_max), ']']);
        disp(['lb 크기: ', num2str(size(lb,1)), 'x', num2str(size(lb,2))]);
        disp(['ub 크기: ', num2str(size(ub,1)), 'x', num2str(size(ub,2))]);

        % 최적화 문제 해결
        disp('=== quadprog 최적화 시작 ===');
        options = optimoptions('quadprog', 'Algorithm', 'interior-point-convex', 'Display', 'off');

        [U_sequence, ~, exitflag, output] = quadprog(H, f, [], [], [], [], lb, ub, [], options);

        % 디버깅: 최적화 결과 출력
        disp(['quadprog 종료 플래그: ', num2str(exitflag)]);
        if exitflag < 0
            warning('최적화 실패: %s', output.message);
            U = U_prev;  % 이전 제어 입력 유지
            disp('이전 제어 입력 사용');
        else
            disp(['최적화 성공: ', output.message]);
            disp(['U_sequence 크기: ', num2str(size(U_sequence,1)), 'x', num2str(size(U_sequence,2))]);

            % 첫 번째 제어 입력 추출
            if length(U_sequence) >= 8
                U = U_sequence(1:8);

                % 디버깅: 초기 제어 입력 출력
                disp('--- 최적화 결과 제어 입력 ---');
                disp(['카나드 각도(rad): ', num2str(U(1:4)')]);
                disp(['카나드 각도(deg): ', num2str(rad2deg(U(1:4)'))]);
                disp(['RCS 상태(연속): ', num2str(U(5:8)')]);

                % 카나드 각도 제한 적용 - 연속적인 값 그대로 사용
                for i = 1:4
                    U(i) = max(canard_min, min(canard_max, U(i)));

                    % 매우 작은 값 처리
                    if abs(U(i)) < 0.01
                        U(i) = sign(U(i) + 1e-10) * 0.015;  % ±0.015 rad (약 ±0.85도)
                    end
                end

                rcs_min_threshold = sum( U(5:8) ) / 4;  % 최소 추력 임계값 [N] - 이 이하면 꺼짐

                % 각 RCS 임계값 적용 (U(i)가 0~3N 범위의 값)
                for i = 5:8
                    % 추력이 임계값 이하면 끄기
                    if U(i) <= rcs_min_threshold
                        U(i) = 0;
                    else
                        U(i) = 1;  % 임계값 초과하면 최대 추력(3N) 사용
                    end
                end
                % 디버깅: 최종 제어 입력 출력
                disp('--- 최종 제어 입력 (이진화 후) ---');
                disp(['카나드 각도(rad): ', num2str(U(1:4)')]);
                disp(['카나드 각도(deg): ', num2str(rad2deg(U(1:4)'))]);
                disp(['RCS 상태(이진): ', num2str(U(5:8)')]);
            else
                warning('U_sequence 길이 부족: %d', length(U_sequence));
                U = U_prev;  % 이전 제어 입력 유지
                disp(['이전 제어 입력 사용: ', num2str(U')]);
            end

            % U가 열벡터인지 확인
            U = reshape(U, 8, 1);
        end

        % 예측 궤적 계산
        predicted_trajectory = zeros(7, Np+1);
        predicted_trajectory(:,1) = x_current;  % 초기 상태 설정

        % 제어 입력 시퀀스 재구성
        U_seq = zeros(8, Np);
        for i = 1:min(Nc, Np)
            if length(U_sequence) >= (i-1)*8+8
                U_seq(:,i) = U_sequence((i-1)*8+1:(i-1)*8+8);
            else
                U_seq(:,i) = U_prev;
            end
        end

        if Nc < Np
            U_seq(:,Nc+1:end) = repmat(U_seq(:,Nc), 1, Np-Nc);
        end

        % 예측 궤적 시뮬레이션
        for i = 1:Np
            predicted_trajectory(:,i+1) = A * predicted_trajectory(:,i) + B * U_seq(:,i);

            % 쿼터니언 정규화
            quat_norm = norm(predicted_trajectory(1:4,i+1));
            if quat_norm > 0
                predicted_trajectory(1:4,i+1) = predicted_trajectory(1:4,i+1) / quat_norm;
            end
        end

        % 디버깅: 예측 궤적 첫 번째와 마지막 상태 출력
        disp('--- 예측 궤적 ---');
        disp(['초기상태: ', num2str(predicted_trajectory(:,1)')]);
        disp(['최종상태: ', num2str(predicted_trajectory(:,end)')]);

    catch ME
        warning('MPC 계산 중 오류: %s\n%s', ME.message, getReport(ME, 'extended'));
        disp('오류 스택:');
        disp(ME.stack(1));

        % 디버깅: 비상 제어 입력 생성
        disp('비상 제어 입력 생성');

        % 간단한 비례 제어 (오차에 비례하는 값)
        try
            quat_error = q2q_mult(inv_q(quat_current), quat_target);
            error_angle = 2 * acos(abs(quat_error(1)));

            % 비상 카나드 제어 (단순 비례 제어)
            canard_scale = 0.1;  % 스케일 계수
            U = zeros(8, 1);

            % 오차 벡터로부터 카나드 각도 설정
            U(1) = canard_scale * quat_error(2);   % 카나드 1 - 요 제어
            U(2) = canard_scale * quat_error(3);   % 카나드 2 - 피치 제어
            U(3) = -canard_scale * quat_error(2);  % 카나드 3 - 요 제어 (반대)
            U(4) = -canard_scale * quat_error(3);   % 카나드 4 - 피치 제어

            % RCS 비상 활성화 - 롤 제어
            % 이제 RCS 1,3은 양의 롤, RCS 2,4는 음의 롤을 생성합니다.
            if omega(1) < 0  % 음의 롤 각속도 -> 양의 롤 모멘트 필요
                U(5) = 1;    % RCS 1 활성화 (+롤)
                U(7) = 1;    % RCS 3 활성화 (+롤)
            else             % 양의 롤 각속도 -> 음의 롤 모멘트 필요
                U(6) = 1;    % RCS 2 활성화 (-롤)
                U(8) = 1;    % RCS 4 활성화 (-롤)
            end

            disp(['비상 제어 입력: ', num2str(U')]);
        catch
            U = U_prev;  % 이전 제어 입력 유지
            disp(['비상 제어 실패, 이전 입력 사용: ', num2str(U')]);
        end

        predicted_trajectory = zeros(7, Np+1);
        predicted_trajectory(1,1) = 1;  % 단위 쿼터니언 (스칼라 부분)
    end

    % 계산 시간 측정
    computation_time = toc;

    % 디버깅: 최종 결과 요약
    disp('======= MPC 제어기 실행 결과 =======');
    disp(['계산 시간: ', num2str(computation_time*1000), ' ms']);
    disp(['제어 입력: ', num2str(U')]);
    disp('====================================');
end