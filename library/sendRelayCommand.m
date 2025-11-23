% 릴레이 명령을 전송하는 함수
function sendRelayCommand(client, relayCommand)
    try
        commandStr = sprintf('%d ', relayCommand);
        write(client, commandStr);
    catch
        disp('Failed to send command to Jetson.');
    end
end