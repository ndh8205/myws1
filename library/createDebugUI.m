function [debugHandles] = createDebugUI()
    global debugData
    
    % 데이터 저장을 위한 구조체 초기화
    debugData.time = [];
    debugData.X_target = [];
    debugData.X_actual = [];
    debugData.Y_target = [];
    debugData.Y_actual = [];
    debugData.Attitude_target = [];
    debugData.Attitude_actual = [];
    debugData.U = [];  % 명령 벡터 초기화
    debugData.relayStates = [];  % 릴레이 상태 초기화
    
    % 디버깅 UI 생성
    figure('Name', 'Debug UI');
    
    % X 위치 vs 시간 플롯 설정
    subplot(2,2,1);
    debugHandles.hX_target = animatedline('Color', 'b', 'DisplayName', 'Target X');
    hold on;
    debugHandles.hX_actual = animatedline('Color', 'r', 'DisplayName', 'Actual X');
    title('X Position vs Time');
    xlabel('Time (s)');
    ylabel('X (m)');
    legend show;
    grid on;
    
    % Y 위치 vs 시간 플롯 설정
    subplot(2,2,2);
    debugHandles.hY_target = animatedline('Color', 'b', 'DisplayName', 'Target Y');
    hold on;
    debugHandles.hY_actual = animatedline('Color', 'r', 'DisplayName', 'Actual Y');
    title('Y Position vs Time');
    xlabel('Time (s)');
    ylabel('Y (m)');
    legend show;
    grid on;
    
    % 자세 (각도) vs 시간 플롯 설정
    subplot(2,2,3);
    debugHandles.hAtt_target = animatedline('Color', 'b', 'DisplayName', 'Target Yaw');
    hold on;
    debugHandles.hAtt_actual = animatedline('Color', 'r', 'DisplayName', 'Actual Yaw');
    title('Attitude (Yaw) vs Time');
    xlabel('Time (s)');
    ylabel('Yaw (degrees)');
    legend show;
    grid on;

    % 명령 벡터 플롯 설정
    subplot(2,2,4);
    debugHandles.hCommandLines = gobjects(8, 1);
    colors = lines(8);  % 8개의 색상 배열
    for i = 1:8
        debugHandles.hCommandLines(i) = animatedline('Color', colors(i,:), 'DisplayName', ['Command ', num2str(i)]);
        hold on;
    end
    title('Command Vectors vs Time');
    xlabel('Time (s)');
    ylabel('State');
    ylim([-0.5, 1.5]);
    yticks([0, 1]);
    yticklabels({'Off', 'On'});
    legend show;
    grid on;
end
