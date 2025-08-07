function [A, B] = linearize_HANul_ana(X, U_prev, params, dt)
% LINEARIZE_HANUL_ANA: HANul 로켓용 심볼릭 선형화 함수
%
% 입력:
% X: 현재 상태 벡터 [13x1] [pos; vel; quat; omega]
% U_prev: 이전 제어 입력 [8x1] [canard1-4; RCS1-4]
% params: 로켓 파라미터 구조체
% dt: 시간 스텝 [s]
%
% 출력:
% A: 이산시간 상태 행렬 (쿼터니언+각속도) [7x7] (스케일링+정규화 적용됨)
% B: 이산시간 입력 행렬 (쿼터니언+각속도) [7x8] (스케일링+정규화 적용됨)

    % 이산 모드로 전체 선형화 수행
    [Ad_full, Bd_full] = linearize_rocket(X, U_prev, params, dt);
    
    % 쿼터니언 및 각속도 관련 부분만 추출 (MPC 제어기용)
    quat_indices = 7:10;    % 쿼터니언 인덱스
    omega_indices = 11:13;  % 각속도 인덱스
    mpc_indices = [quat_indices, omega_indices]; % 전체 MPC 상태 인덱스
    
    % 필요한 부분만 추출 (이산 시스템 행렬)
    A_raw = Ad_full(mpc_indices, mpc_indices);
    B_raw = Bd_full(mpc_indices, :);
    
    % 수치적 안정성을 위한 작은 값 제거
    A_raw(abs(A_raw) < 1e-10) = 0;
    B_raw(abs(B_raw) < 1e-10) = 0;
    
    % ===== 1. 스케일링 적용 =====
    
    % 상태 스케일링 벡터 정의
    % 쿼터니언은 이미 [-1,1] 범위라 1로 유지, 각속도는 0.1 적용하여 스케일 균형
    state_scaling = [1; 1; 1; 1; 0.1; 0.1; 0.1];
    
    % 입력 스케일링 벡터 정의
    % 카나드 각도는 라디안 단위이므로 10 적용, RCS는 바이너리 값이라 1로 유지
    input_scaling = [10; 10; 10; 10; 1; 1; 1; 1];
    
    % 행렬 스케일링 적용
    Sx = diag(state_scaling);
    Sx_inv = diag(1./state_scaling);
    Su = diag(input_scaling);
    
    A_scaled = Sx_inv * A_raw * Sx;
    B_scaled = Sx_inv * B_raw * Su;
    
    % ===== 2. 정규화 적용 =====
    
    % A 행렬 조건수 확인
    cond_A = cond(A_scaled);
    
    % A 행렬 조건수가 높으면 SVD 기반 정규화 적용
    if cond_A > 1e10
        % 정규화 매개변수 설정
        lambda = 1e-6;
        
        % 방법 1: Tikhonov 정규화 (릿지 정규화)
        % A_reg = A_scaled + lambda * eye(size(A_scaled));
        
        % 방법 2: SVD 기반 정규화 (더 강력한 방법)
        [U, S, V] = svd(A_scaled);
        s = diag(S);
        s_max = max(s);
        
        % 너무 작은 특이값 제한
        tol = s_max * 1e-10;
        s_reg = max(s, tol);
        
        % 정규화된 A 행렬 재구성
        A_reg = U * diag(s_reg) * V';
        
        % 결과 할당
        A = A_reg;
        B = B_scaled; % B는 일반적으로 정규화할 필요 없음
        
        % 디버그 정보 (필요시)
        fprintf('A 행렬 정규화 적용됨: 조건수 %.2e → %.2e\n', cond_A, cond(A_reg));
    else
        % 조건수가 허용 범위 내면 스케일링만 적용
        A = A_scaled;
        B = B_scaled;
    end
    
    % ===== 3. 행렬 균형화 (필요시) =====
    
    % 행/열 균형화를 위한 추가 처리 (선택 사항)
    if cond(A) > 1e8
        % 행렬 균형화를 위한 함수 (MATLAB 내장)
        try
            [T, A_balanced] = balance(A);
            A = A_balanced;
            B = T \ B; % 균형 잡힌 시스템에 맞게 B 조정
            fprintf('행렬 균형화 적용됨: 조건수 %.2e → %.2e\n', cond_A, cond(A));
        catch
            % 균형화 실패 시 원래 행렬 사용
            fprintf('행렬 균형화 실패, 원래 행렬 사용\n');
        end
    end
    
    % 최종 결과에서 수치적 노이즈 제거
    A(abs(A) < 1e-10) = 0;
    B(abs(B) < 1e-10) = 0;
    
    % 디버그 정보: 최종 행렬 정보
    fprintf('최종 A 행렬 조건수: %.2e\n', cond(A));
end