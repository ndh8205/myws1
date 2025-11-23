function updateUI(global_translation, posX, posY, RZz)
    global hRawPos hFilteredPos hRawCurrentPos hFilteredCurrentPos hPolarArrow hAngleVsTime hPrevTime hRelayHeatmap U_binary_global
    
    % 실시간 위치 업데이트
    
    addpoints(hRawPos, global_translation(1), global_translation(2));
    set(hRawCurrentPos, 'XData', global_translation(1), 'YData', global_translation(2));
    addpoints(hFilteredPos, posX, posY);
    set(hFilteredCurrentPos, 'XData', posX, 'YData', posY);
    


    % 실시간 자세 업데이트 (폴라 플롯)
    angle = RZz;
    arrowLength = 1;
    set(hPolarArrow, 'ThetaData', [angle, angle], 'RData', [0, arrowLength]);

    % 각도 vs 시간 플롯 업데이트
    currentTime = toc(hPrevTime);
    addpoints(hAngleVsTime, currentTime, rad2deg(angle));

    heatmapData = get(hRelayHeatmap, 'CData');
    relayPositions = [
                        2,19;  7,28; 19,28; 28,19;  28,7; 19,2;  7,2; 2,7;
                        2,20;  8,28; 20,28; 28,20;  28,8; 20,2;  8,2; 2,8;
                        2,21;  9,28; 21,28; 28,21;  28,9; 21,2;  9,2; 2,9;
                        2,22; 10,28; 22,28; 28,22; 28,10; 22,2; 10,2; 2,10;
                        2,23; 11,28; 23,28; 28,23; 28,11; 23,2; 11,2; 2,11
                     ];

    for i = 1 : 40
        row = relayPositions(i, 1);
        col = relayPositions(i, 2);
        if U_binary_global(mod(i-1, 8) + 1) == 1
            heatmapData(row, col, :) = [255, 0, 0] / 255;
        else
            heatmapData(row, col, :) = [23, 155, 174] / 255;
        end
    end
    set(hRelayHeatmap, 'CData', heatmapData);

    drawnow;
end