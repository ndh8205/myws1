function [U, computation_time] = Controller_RL_NMPC(X, X_target, dt, params)
    % RL-NMPC 매개변수
    stateDimension = 6;  % 상태 차원
    actionDimension = 8;  % 행동 차원 (제어 입력)

    % 관측 및 행동 정보 정의
    obsInfo = rlNumericSpec([stateDimension 1]);
    obsInfo.Name = 'observations';
    actInfo = rlNumericSpec([actionDimension 1], 'LowerLimit', 0, 'UpperLimit', 1);
    actInfo.Name = 'actions';

    % 환경 생성
    env = rlFunctionEnv(obsInfo, actInfo, @(action, state) stepFunction(action, state, X_target, dt, params));

    % 신경망 구조 정의
    statePath = [
        imageInputLayer([stateDimension 1 1], 'Normalization', 'none', 'Name', 'state')
        fullyConnectedLayer(64, 'Name', 'fc1')
        reluLayer('Name', 'relu1')
        fullyConnectedLayer(64, 'Name', 'fc2')
        reluLayer('Name', 'relu2')
        fullyConnectedLayer(actionDimension, 'Name', 'fc3')
    ];

    criticPath = [
        imageInputLayer([stateDimension 1 1], 'Normalization', 'none', 'Name', 'state')
        fullyConnectedLayer(64, 'Name', 'fc1')
        reluLayer('Name', 'relu1')
        fullyConnectedLayer(64, 'Name', 'fc2')
        reluLayer('Name', 'relu2')
        fullyConnectedLayer(1, 'Name', 'fc3')
    ];

    % DDPG 에이전트 생성
    agent = rlDDPGAgent(actor(statePath, obsInfo, actInfo), critic(criticPath, obsInfo, actInfo));

    % 훈련 옵션 설정
    trainOpts = rlTrainingOptions(...
        'MaxEpisodes', 1000, ...
        'MaxStepsPerEpisode', 500, ...
        'ScoreAveragingWindowLength', 100, ...
        'Verbose', false, ...
        'Plots', 'training-progress');

    % 에이전트 훈련
    trainingStats = train(agent, env, trainOpts);

    % 훈련된 에이전트를 사용하여 최적 제어 입력 계산
    tic;
    action = getAction(agent, X);
    computation_time = toc;

    U = reshape(action, [actionDimension, 1]);

    disp(['Computation time: ', num2str(computation_time), ' seconds']);
end

function [nextObs, reward, isDone, loggedSignals] = stepFunction(action, state, X_target, dt, params)
    % 시스템 다이나믹스 시뮬레이션
    nextState = rk4(@vehicle_dynamics_Airbearing, state, action, dt, params);
    
    % 보상 계산
    stateError = norm(nextState - X_target);
    controlEffort = norm(action);
    reward = -stateError - 0.1 * controlEffort;
    
    % 종료 조건
    isDone = (stateError < 0.1) || (norm(nextState) > 1000);
    
    nextObs = nextState;
    loggedSignals = [];
end