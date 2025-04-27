clear all
close all
clc

% ROS2 환경 변수 설정 - 도메인 ID 51
setenv('ROS_DOMAIN_ID', '51');
setenv('ROS_LOCALHOST_ONLY', '0');
setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp');

% 잠시 대기
pause(2);

% ROS2 노드 생성
try
    ros2 = ros2node('matlab_listener');
    disp('ROS2 노드가 성공적으로 생성되었습니다.');
    
    % 구독자 설정 - /chatter 토픽 (talker 노드가 발행)
    sub = ros2subscriber(ros2, '/chatter', 'std_msgs/String');
    sub.NewMessageFcn = @(~, msg) disp(['ROS2 메시지 수신: ', msg.data]);
    disp('토픽 구독이 설정되었습니다. 메시지를 기다리는 중...');
    
    % 발행자 설정 - MATLAB에서 WSL2로 메시지 전송
    pub = ros2publisher(ros2, '/matlab_topic', 'std_msgs/String');
    msg = ros2message('std_msgs/String');
    msg.data = 'Hello from MATLAB!';
    
    % 메시지 발행
    for i = 1:5
        send(pub, msg);
        disp(['메시지 발행: ', msg.data]);
        pause(1);
    end
    
    % 노드 목록 확인
    nodes = ros2('node', 'list');
    disp('발견된 노드 목록:');
    disp(nodes);
    
catch ME
    disp(['오류 발생: ', ME.message]);
end