function [F, G] = mpc_prediction_matrices(A, B, Np, Nc)
    % mpc_prediction_matrices: MPC 예측 행렬 구성
    %
    % 입력:
    % A: 이산화된 시스템 행렬 (상태 방정식) (7x7)
    % B: 이산화된 입력 행렬 (상태 방정식) (7x8)
    % Np: 예측 지평선 (prediction horizon)
    % Nc: 제어 지평선 (control horizon)
    %
    % 출력:
    % F: 자유 응답 행렬 - 현재 상태의 영향 (7*Np x 7)
    % G: 제어 응답 행렬 - 제어 입력의 영향 (7*Np x 8*Nc)
    
    % 상태 및 입력 차원 추출
    [nx, ~] = size(A);  % 상태 차원
    [~, nu] = size(B);  % 제어 입력 차원
    
    % 디버깅: 행렬 크기 출력
    disp(['mpc_prediction_matrices - 상태 차원: ', num2str(nx), ', 입력 차원: ', num2str(nu)]);
    
    % 자유 응답 행렬 (현재 상태의 영향)
    F = zeros(nx*Np, nx);
    for i = 1:Np
        F((i-1)*nx+1:i*nx, :) = A^i;
    end
    
    % 제어 응답 행렬 (제어 입력의 영향)
    G = zeros(nx*Np, nu*Nc);
    for i = 1:Np
        for j = 1:min(i, Nc)
            row_idx = (i-1)*nx+1:i*nx;
            col_idx = (j-1)*nu+1:j*nu;
            G(row_idx, col_idx) = A^(i-j) * B;
        end
    end
    
    % 디버깅: 행렬 크기 확인
    disp(['F 행렬 크기: ', num2str(size(F,1)), 'x', num2str(size(F,2))]);
    disp(['G 행렬 크기: ', num2str(size(G,1)), 'x', num2str(size(G,2))]);
end