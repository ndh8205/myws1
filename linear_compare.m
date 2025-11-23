% Initialize Matlab
clc;
clear all;
close all;

% Add Library directory
addpath(genpath('C:\Users\USER\Desktop\Kalman_Filter\Sat_ver2\main'));

% Initialize setting
params = Params_init(); % Load Parameter struct

X = [ 517; 484; 0; 0; 0; 0 ]; % ( posX, posY, V_X, V_Y, rotZ, rateZ )
Master_point = [ 2400; 2400; 0; 0; 0; 0 ]; % Initialize Mothership position and orientation
X_AB = X;
X_linear = X;
xl = X;

% Simulation setting
Main_Hz = params.System.MainLoopHz;
Sensor_VI_Hz = params.Sensor.VICON_HZ;
PID_control_Hz = Main_Hz;
OPENCV_HZ = params.Sensor.OPENCV_HZ;

sim_dt = 1 / Main_Hz; % [sec]
sim_Time = 160; % [sec]
sim_step = ceil( sim_Time / sim_dt );

% Sensor & Control interval Setting
Sensor_interval = Main_Hz / OPENCV_HZ; % sensor sampling set up
PID_control_interval = Main_Hz / PID_control_Hz;

% Time duty initialize
real_time = 0;

% Data log initialize
state_history = zeros(6, sim_step);
state_history_AB = zeros(6, sim_step);
t = zeros(1, sim_step);
target_history = zeros(6, sim_step);
input_history = zeros(9, sim_step); % 제어 입력 저장을 위한 변수 (U는 9x1 벡터)
X_target_history = zeros(6, sim_step); % 목표 상태 저장

% LSTM 학습을 위한 데이터 저장 변수 초기화
input_data = [];
output_data = [];

% Main simulation loop
for i = 1 : sim_step
    
    X_target = generate_commands(real_time); % Generate - X_target & tolerance
    X_target_history(:, i) = X_target; % 목표 상태 저장

    if mod(i-1, PID_control_interval) == 0 

        U = Controller_PID_Argument( X, X_target, sim_dt, params );

    end

    [ final_cmd, debug_cmd_delta ]  = Control_Allocator( U, params );

    final_cmd(9) = cmd2tq_origin( final_cmd(9), 49, params, sim_dt );

    % Stochastic RK4 simulation
    [ X_next, Fskr4 ] = srk4_debug( @vehicle_dynamics_Airbearing_stochastic_debug, X, final_cmd, zeros(3,1), sim_dt, params, sim_dt );
    [ X_AB_next, FAB ] = rk4_debug( @vehicle_dynamics_Airbearing_AXBU, X_AB, final_cmd, sim_dt, params );
    FAB = [ FAB(3); FAB(4); FAB(6) ];

    % 데이터 저장
    state_history(:, i) = X;
    state_history_AB(:, i) = X_AB;
    t(i) = real_time;
    target_history(:, i) = X_target;
    input_history(:, i) = final_cmd; % 제어 입력 저장 (9개의 입력)

    % LSTM 학습용 데이터 수집
    input_data = [input_data; [X; final_cmd]']; % 입력 데이터 (현재 상태와 제어 입력)
    output_data = [output_data; X_next']; % 출력 데이터 (다음 상태)

    % 상태 업데이트
    X = X_next;
    X_AB = X_AB_next;

    real_time = real_time + sim_dt;

end

%% LSTM 모델 학습

% 시퀀스 길이 설정
seqLength = 10;

% 사용할 수 있는 시퀀스 수 계산
numSequences = floor(length(input_data)/seqLength);

% 데이터 자르기
input_data = input_data(1:numSequences*seqLength, :);
output_data = output_data(1:numSequences*seqLength, :);

% 데이터를 시퀀스로 재구성
input_data_seq = reshape(input_data', size(input_data, 2), seqLength, numSequences);
output_data_seq = reshape(output_data', size(output_data, 2), seqLength, numSequences);

% 셀 배열로 변환
inputSequence = cell(1, numSequences);
outputSequence = cell(1, numSequences);

for i = 1:numSequences
    inputSequence{i} = input_data_seq(:, :, i);
    outputSequence{i} = output_data_seq(:, :, i);
end

% 학습 및 검증 데이터 분할
numTrain = floor(0.8 * numSequences);
XTrain = inputSequence(1:numTrain);
YTrain = outputSequence(1:numTrain);
XValidation = inputSequence(numTrain+1:end);
YValidation = outputSequence(numTrain+1:end);

% LSTM 네트워크 아키텍처 정의
numFeatures = size(input_data, 2); % 입력의 특성 수 (상태 6 + 제어 입력 9 = 15)
numResponses = size(output_data, 2); % 출력의 수 (다음 상태 6)
numHiddenUnits = 100;

layers = [
    sequenceInputLayer(numFeatures)
    lstmLayer(numHiddenUnits, 'OutputMode', 'sequence')
    fullyConnectedLayer(numResponses)
    regressionLayer
];

% 학습 옵션 설정
options = trainingOptions('adam', ...
    'MaxEpochs', 50, ...
    'MiniBatchSize', 20, ...
    'InitialLearnRate', 0.001, ...
    'ValidationData', {XValidation, YValidation}, ...
    'Plots', 'training-progress', ...
    'Verbose', false);

% 네트워크 학습
net = trainNetwork(XTrain, YTrain, layers, options);

%% LSTM 모델을 사용한 시뮬레이션

% LSTM을 사용한 상태 예측 변수 초기화
state_history_LSTM = zeros(6, sim_step);

% 초기 상태 설정
X_LSTM = state_history(:, 1);
U_LSTM = input_history(:, 1);

% LSTM 모델을 사용한 시뮬레이션에서 입력 시퀀스 수정
input_seq = zeros(numFeatures, seqLength);
for i = 1:seqLength
    if i <= size(input_history, 2)
        input_seq(:, i) = [state_history(:, i); input_history(:, i)]; % 수정된 부분
    else
        input_seq(:, i) = zeros(numFeatures, 1);
    end
end

for i = 1 : sim_step - seqLength
    
    % LSTM을 사용하여 다음 상태 예측
    X_input = input_seq; % 현재 입력 시퀀스
    X_pred = predict(net, X_input, 'ExecutionEnvironment', 'cpu');
    X_LSTM = X_pred(:, end); % 마지막 타임스텝의 예측 상태

    % 데이터 저장
    state_history_LSTM(:, i+seqLength) = X_LSTM;

    % 실제 시스템과 동일한 제어 입력 사용
    if mod(i-1, PID_control_interval) == 0
        X_target = X_target_history(:, i+seqLength);
        U_LSTM = Controller_PID_Argument(X_LSTM, X_target, sim_dt, params);
        [ U_LSTM, debug_cmd_delta_lstm ]  = Control_Allocator( U_LSTM, params );
    end

    % 입력 시퀀스 업데이트
    input_seq = [input_seq(:, 2:end), [X_LSTM' U_LSTM']'];

end

%% Plot the results

figure(1);
subplot(3, 1, 1);
plot(t, state_history(1, :), 'b', t, target_history(1, :), 'r--', t, state_history_AB(1, :), 'm--', t(seqLength+1:end), state_history_LSTM(1, seqLength+1:end), 'g--');
xlabel('[sec]');
ylabel('[mm]');
title('X Position');
legend('Nonlinear', 'Target', 'Linear', 'LSTM');

subplot(3, 1, 2);
plot(t, state_history(2, :), 'b', t, target_history(2, :), 'r--', t, state_history_AB(2, :), 'm--', t(seqLength+1:end), state_history_LSTM(2, seqLength+1:end), 'g--');
xlabel('[sec]');
ylabel('[mm]');
title('Y Position');
legend('Nonlinear', 'Target', 'Linear', 'LSTM');

subplot(3, 1, 3);
plot(t, rad2deg(state_history(5, :)), 'b', t, rad2deg(target_history(5, :)), 'r--', t, rad2deg(state_history_AB(5, :)), 'm--', t(seqLength+1:end), rad2deg(state_history_LSTM(5, seqLength+1:end)), 'g--');
xlabel('[sec]');
ylabel('[deg]');
title('Yaw Angle (psi)');
legend('Nonlinear', 'Target', 'Linear', 'LSTM');
