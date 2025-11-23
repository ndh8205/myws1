% 모터 명령을 전송하는 함수
function sendMotorCommand(client, motorSpeed)
    try
        % motorSpeed는 0에서 100 사이의 값이어야 합니다.
        if motorSpeed < 0 || motorSpeed > 100
            error('Motor speed must be between 0 and 100.');
        end
        commandStr = sprintf('M %d', motorSpeed);
        write(client, uint8(commandStr), 'char');
    catch ME
        disp('Failed to send command to Jetson.');
        disp(ME.message);
    end
end