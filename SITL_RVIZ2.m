clc;
clear all;
close all;

% Add Library directory
addpath(genpath('C:\Users\DDHD\Desktop\shj\main'));

% Initialize setting
params = Params_init(); % Load Parameter struct

X = [ 517; 484; 0; 0; 0; 0 ]; % ( posX, posY, V_X, V_Y, rotZ, rateZ )
master_point = [2400; 2400; 0; 0; deg2rad(90); 0]; % Initialize Mothership position and orientation
X_true = X;
Xk = X;
z = X; % ( posX, posY, V_X, V_Y, rotZ, rateZ )
final_cmd = [ zeros( 8,1 ); 49 ]; % ( T1 T2 T3 T4 T5 T6 T7 T8 RW )

P = params.EKF.P;
Rk = params.EKF.R;
Rm = params.EKF.Rm;

Q_Fx = 1;
Q_Fy = 1;
Q_Tau = 1;
Qw = diag( [ Q_Fx.^2, Q_Fx.^2, Q_Tau.^2 ]' );

% ROS2 환경 설정
ros2_ip = '172.21.126.136';
setenv('ROS_DOMAIN_ID', '0');
setenv('ROS_LOCALHOST_ONLY', '0');
setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp');

% ROS2 노드 생성
ros2 = ros2node('matlab_node');
disp('ROS2 노드가 성공적으로 생성되었습니다.');
disp(['노드 이름: ', ros2.Name]);

% 테스트용 발행자 생성
pub = ros2publisher(ros2, '/test_topic', 'std_msgs/String');
disp(['발행자 토픽: ', pub.TopicName]);

% 테스트용 구독자 생성
sub = ros2subscriber(ros2, '/test_topic', @messageCallback);
disp(['구독자 토픽: ', sub.TopicName]);

% Marker 퍼블리셔 생성
markerPub = ros2publisher(ros2, '/visualization_marker', 'visualization_msgs/Marker');
disp(['Marker 퍼블리셔 토픽: ', markerPub.TopicName]);

% Path 퍼블리셔 생성
pathPub = ros2publisher(ros2, '/robot_path', 'nav_msgs/Path');
disp(['Path 퍼블리셔 토픽: ', pathPub.TopicName]);

% Marker 메시지 초기화
marker = ros2message(markerPub);
marker.Header.FrameId = 'map'; % 기준 프레임 설정
marker.Type = 2; % ARROW 타입
marker.Action = 0; % ADD
marker.Scale.X = 0.5; % 화살표 길이
marker.Scale.Y = 0.1; % 화살표 너비
marker.Scale.Z = 0.1;
marker.Color.R = 0.0;
marker.Color.G = 1.0; % 녹색
marker.Color.B = 0.0;
marker.Color.A = 1.0; % 불투명
marker.Id = 0; % Marker ID

% Path 메시지 초기화
pathMsg = ros2message(pathPub);
pathMsg.Header.FrameId = 'map'; % 기준 프레임 설정

% 주기적으로 메시지 발행
disp('5초 간격으로 메시지를 발행합니다. Ctrl+C로 중지할 수 있습니다.');
count = 1;

% Simulation setting
Main_Hz = params.System.MainLoopHz;
Sensor_VI_Hz = params.Sensor.VICON_HZ;
PID_control_Hz = Main_Hz;
AHRS_HZ = params.Sensor.AHRS_HZ;
OPENCV_HZ = params.Sensor.OPENCV_HZ;

sim_dt = 1 / Main_Hz; % [sec]
sim_Time = 160; % [sec]
sim_step = ceil(sim_Time / sim_dt);

% Sensor & Control interval Setting
Sensor_VI_interval = Main_Hz / OPENCV_HZ; % sensor sampling set up
PID_control_interval = Main_Hz / PID_control_Hz; 

% Time duty initialize
real_time = 0;

% Data log initialize
state_history = zeros(6, sim_step);
t = zeros(1, sim_step);
target_history = zeros(6, sim_step);

% Main simulation loop
for i = 1 : sim_step

    tic;
    
    X_target = generate_commands(real_time, Xk);

    if mod(i-1, Sensor_VI_interval) == 0 

        [ z_AHRS, z_AHRS_true ] = measure_sensor_AHRS( Xk, master_point, Rm );
        [ z_Aruco, z_Aruco_true ] = measure_sensor_Aruco( Xk, master_point, Rm );

        z = [ z_Aruco; z_AHRS ];
        z_true = [ z_Aruco_true; z_AHRS_true ];

        [Xk, P, yhat, X_predict] = processDataSIM_Muti_EKF(z, P, Xk, master_point, final_cmd, params, sim_dt);

        % 상태 필터 기록
        X_rel_I = master_point(1:2) - Xk(1:2);
        X_rel_B = [cos(Xk(5)), sin(Xk(5)); -sin(Xk(5)), cos(Xk(5))] * X_rel_I;
        rho_k = sqrt(X_rel_B(1)^2 + X_rel_B(2)^2);
        theta_k = atan2(X_rel_B(2), X_rel_B(1));
        r_k = Xk(6);

        % 측정 벡터
        h1 = [ rho_k; theta_k; r_k ];

    end

    if mod(i-1, PID_control_interval) == 0 

        U = Controller_PID_Argument(Xk, X_target, sim_dt, params);

    end

    [ final_cmd, debug_cmd_delta ]  = Control_Allocator(U, params);
    
    % 확률적 RK4 시뮬레이션
    Xk = srk4(@vehicle_dynamics_Airbearing_stochastic, Xk, final_cmd, Qw, sim_dt, params, sim_dt);

    % 데이터 저장
    state_history(:, i) = Xk;
    t(i) = real_time;
    target_history(:, i) = X_target;
      
    real_time = real_time + sim_dt;

    sim_timer = toc;
    
    % Marker 업데이트
    marker.Header.Stamp = rostime('now'); % 현재 시간 설정
    marker.Pose.Position.X = Xk(1) / 1000; % mm to meters
    marker.Pose.Position.Y = Xk(2) / 1000;
    marker.Pose.Position.Z = 0;
    marker.Pose.Orientation = eul2quat([0, 0, Xk(5)]); % Yaw만 설정
    marker.Scale.X = 0.5;
    marker.Scale.Y = 0.1;
    marker.Scale.Z = 0.1;
    
    % Marker 퍼블리시
    send(markerPub, marker);
    
    % Path 업데이트 및 퍼블리시 (선택 사항)
    poseStamped = ros2message('geometry_msgs/PoseStamped');
    poseStamped.Header.Stamp = rostime('now');
    poseStamped.Header.FrameId = 'map';
    poseStamped.Pose.Position.X = Xk(1) / 1000;
    poseStamped.Pose.Position.Y = Xk(2) / 1000;
    poseStamped.Pose.Position.Z = 0;
    poseStamped.Pose.Orientation = eul2quat([0, 0, Xk(5)]);
    
    pathMsg.Poses = [pathMsg.Poses, poseStamped];
    send(pathPub, pathMsg);
    
end

% 콜백 함수 정의
function messageCallback(message)
    disp('메시지를 수신했습니다:');
    disp(message.Data);
end

% Helper function to convert Euler angles to Quaternion
function quat = eul2quat(eul)
    % eul: [roll, pitch, yaw]
    q = quaternion(eul(1), eul(2), eul(3), 'eulerd', 'XYZ');
    quat = rotmat2quat(rotm(eul));
end

function R = rotm(eul)
    % eul: [roll, pitch, yaw]
    roll = eul(1);
    pitch = eul(2);
    yaw = eul(3);
    R_x = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
    R_y = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
    R_z = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
    R = R_z * R_y * R_x;
end
