function error = calculate_rmse(true_state, estimated_state)
    % true_state와 estimated_state는 (상태 차원 x 시간 스텝 수) 크기의 행렬입니다.
    position_error = true_state(1:2, :) - estimated_state(1:2, :);
    rmse = sqrt(mean(sum(position_error.^2, 1)));
    error = rmse;
end
