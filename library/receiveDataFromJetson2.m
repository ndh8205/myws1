function data = receiveDataFromJetson2(client)
    data = '';
    try
        timeout = 1; % 5초 타임아웃
        startTime = tic;
        
        while toc(startTime) < timeout
            if client.BytesAvailable > 0
                chunk = read(client, client.BytesAvailable);
                data = [data, char(chunk)];
                
                % 완전한 JSON 객체를 받았는지 확인
                if contains(data, '}\n{')
                    break;
                end
            else
            end
        end
        
        if toc(startTime) >= timeout
            warning('Data reception timed out.');
        end
    catch ME
        disp(['Error receiving data from Jetson: ', ME.message]);
    end
    
    if isempty(data)
        disp('No data received from Jetson.');
    else
        disp(['Received data: ', data]);
    end
end