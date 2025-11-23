function updateDebugUI(debugHandles, time, X_target, X_actual, Y_target, Y_actual, Attitude_target, Attitude_actual, U)
    % X 위치 업데이트
    addpoints(debugHandles.hX_target, time, X_target);
    addpoints(debugHandles.hX_actual, time, X_actual);
    
    % Y 위치 업데이트
    addpoints(debugHandles.hY_target, time, Y_target);
    addpoints(debugHandles.hY_actual, time, Y_actual);
    
    % 자세 업데이트
    addpoints(debugHandles.hAtt_target, time, rad2deg(Attitude_target));
    addpoints(debugHandles.hAtt_actual, time, rad2deg(Attitude_actual));
    
    % 명령 벡터 업데이트
    for i = 1:8
        addpoints(debugHandles.hCommandLines(i), time, U(i));
    end

    drawnow;
end
