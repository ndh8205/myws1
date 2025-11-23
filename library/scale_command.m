function scaled_value = scale_command(input_value, min_input, max_input, idle_value)

    max_scaled_value = 98;
    idle_torque = 0;

    % 입력값을 -1부터 1 사이로 정규화
    normalized_input = (input_value - idle_torque) / (max_input - min_input);
    
        % 스케일링된 값을 계산
    if normalized_input >= 0
        % 양수 토크인 경우
        scaled_value = idle_value + normalized_input * (max_scaled_value - idle_value);
    else
        % 음수 토크인 경우
        scaled_value = idle_value + normalized_input * idle_value;
    end

    % 스케일링된 값을 정수로 변환하고 범위를 0에서 98로 제한
    scaled_value = max(0, min(98, round(scaled_value)));
end