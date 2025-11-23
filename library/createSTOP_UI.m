function createSTOP_UI()
    global stopFlag;
    fig = uifigure('Position', [100 100 200 100], 'Name', 'Control Panel');
    btn = uibutton(fig, 'push', 'Text', 'Stop', ...
                   'Position', [50 40 100 30], ...
                   'ButtonPushedFcn', @(btn,event) setStopFlag());
end

