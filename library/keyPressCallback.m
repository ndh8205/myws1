function keyPressCallback(~, event)
    global client;
    switch event.Key
        case 'uparrow'
            sendRelayCommand(client, [1, 1, 1, 0, 0, 1, 1, 1]); % 커맨드 예시
        case 'downarrow'
            sendRelayCommand(client, [0, 0, 0, 1, 1, 0, 0, 0]); % 커맨드 예시
        case 'leftarrow'
            sendRelayCommand(client, [1, 0, 0, 0, 0, 0, 0, 1]); % 커맨드 예시
        case 'rightarrow'
            sendRelayCommand(client, [0, 1, 1, 0, 0, 1, 1, 0]); % 커맨드 예시
    end
end