function [X, P, total_latency, interval, rawdata, X_Predict] = EKF_multi_1(client, X, Ut, params, dt, prevTime)
    % Initialize persistent variables
    persistent previous_att;
    persistent previous_Pos;
    persistent previous_rawdata;
    persistent previous_P;

    % Default Value set
    total_latency = 0;
    interval = 0;
    rawdata_init = [716; -1291; 0; 0; deg2rad(0); 0];
    P_init = params.EKF.P;
    X_Predict = X;

    if isempty(previous_rawdata)
        rawdata = rawdata_init;
        previous_rawdata = rawdata;
    else
        rawdata = previous_rawdata;
    end

    if isempty(previous_P)
        P = P_init;
        previous_P = P;
    else
        P = previous_P;
    end

    % Receive Data
    data = receiveDataFromJetson(client);

    if isempty(data)
        rawdata = previous_rawdata;
        P = previous_P;
        return;
    end

    try
        % 수신된 데이터를 줄 단위로 분할
        jsonLines = strsplit(char(data'), '\n');

        for i = 1:length(jsonLines)
            jsonData = strtrim(jsonLines{i});  % 데이터 문자열로 변환 및 공백 제거

            if isempty(jsonData)
                continue;
            end

            parsedData = jsondecode(jsonData);

            % 타임스탬프 및 데이터 처리
            if isfield(parsedData, 'timestamp')
                timestamp_C = posixtime(datetime('now')) * 1000; % 밀리초 단위
                total_latency = timestamp_C - parsedData.timestamp;

                % 수신 주기 측정
                interval = toc(prevTime);

                % VICON 데이터 처리
                if isfield(parsedData, 'subjects') && ~isempty(parsedData.subjects)
                    subject = parsedData.subjects(1);  % 첫 번째 subject
                    segment = subject.segments(1);  % 첫 번째 segment

                    global_translation = segment.global_translation{1};  % 위치 정보
                    global_rotation = segment.global_rotation{1};  % 회전 정보

                    rawdata = [global_translation(1); global_translation(2); global_translation(3);
                               global_rotation(1); global_rotation(2); global_rotation(3)];

                    if isempty(previous_Pos)
                        previous_Pos = zeros(2, 1);
                        previous_att = 0;
                    end

                    Vel_now = (rawdata(1:2) - previous_Pos) / dt;
                    rate_now = (rawdata(6) - previous_att) / dt;

                    previous_Pos = rawdata(1:2);
                    previous_att = rawdata(6);

                % Aruco 데이터 처리
                z_aruco = [];
                if isfield(parsedData, 'aruco') && ~isempty(parsedData.aruco)
                    arucoData = parsedData.aruco;
                    markerIDs = fieldnames(arucoData);
                    % 사용하고자 하는 마커 ID를 지정 (예: '50', '51', '278')
                    targetMarkerIDs = {'x50', 'x51', 'x278'};
                    for i = 1:length(targetMarkerIDs)
                        markerID = targetMarkerIDs{i};
                        if isfield(arucoData, markerID)
                            markerData = arucoData.(markerID);
                            ARtranslation = markerData.translation * 1000;  % [x, y, z]
                            AR_euler_angles = markerData.euler_angles;  % 구조체로 roll, pitch, yaw (degree)
                            pitch_angle = deg2rad(AR_euler_angles.pitch);  % yaw 각도를 라디안으로 변환
                
                            % 필요한 경우 측정 벡터 z_aruco를 구성
                            aruco_norm = sqrt( ARtranslation(1)^2 + ARtranslation(3)^2 );
                            z_aruco = [aruco_norm; pitch_angle];
                        end
                    end
                end

                % AHRS 데이터 처리
                z_ahrs = [];
                if isfield(parsedData, 'euler_rate') && ~isempty(parsedData.euler_rate)
                    ahrsEulerRate = parsedData.euler_rate;
                    rollRate = deg2rad(ahrsEulerRate.roll);
                    pitchRate = deg2rad(ahrsEulerRate.pitch);
                    yawRate = deg2rad(ahrsEulerRate.yaw);
                    % 측정 벡터 z_ahrs 구성
                    z_ahrs = [rollRate; pitchRate; yawRate];
                end

                    % 측정 벡터 z 구성
                    z_VICON = [rawdata(1); rawdata(2); rawdata(6)];

                    z = [ z_aruco; z_ahrs(3) ];

                    previous_rawdata = rawdata;

                    % EKF 업데이트
                    % 필터 파라미터 설정
                    Rq_rho = 1.1;
                    Rq_theta = deg2rad(1000);
                    Rq_rm = deg2rad(1000);

                    qs_u = 20;
                    qs_v = 20;
                    qs_r = deg2rad(10);

                    Rs = diag([Rq_rho^2, Rq_theta^2, Rq_rm^2]);
                    R = Rs;

                    % 상태 전이 행렬 (비선형)
                    F_non = [
                        1, 0, dt * cos(X(5)), -dt * sin(X(5)), 0, 0;
                        0, 1, dt * sin(X(5)),  dt * cos(X(5)), 0, 0;
                        0, 0, 1, -dt * X(6), 0, 0;
                        0, 0, dt * X(6), 1, 0, 0;
                        0, 0, 0, 0, 1, dt;
                        0, 0, 0, 0, 0, 1
                    ];

                    qs_x = 0;
                    qs_y = 0;
                    qs_psi = deg2rad(0);

                    Qs = diag([qs_x^2, qs_y^2, qs_u^2, qs_v^2, qs_psi^2, qs_r^2]);

                    q_x = Qs(1,1);
                    q_y = Qs(2,2);
                    q_u = Qs(3,3);
                    q_v = Qs(4,4);
                    q_psi = Qs(5,5);
                    q_r = Qs(6,6);

                    % 프로세스 노이즈 공분산 Qk 계산
                    Qk = zeros(6,6);

                    Qk(1,1) = dt^3*((q_v*sin(X(5))^2)/3 - (q_u*(sin(X(5))^2 - 1))/3);
                    Qk(1,2) = (dt^3*sin(2*X(5))*(q_u - q_v))/6;
                    Qk(1,3) = (X(6)*q_v*sin(X(5))*dt^3)/3 + (q_u*cos(X(5))*dt^2)/2;
                    Qk(1,4) = (X(6)*q_u*cos(X(5))*dt^3)/3 - (q_v*sin(X(5))*dt^2)/2;

                    Qk(2,1) = Qk(1,2);
                    Qk(2,2) = dt^3*((q_u*sin(X(5))^2)/3 - (q_v*(sin(X(5))^2 - 1))/3);
                    Qk(2,3) = - (X(6)*q_v*cos(X(5))*dt^3)/3 + (q_u*sin(X(5))*dt^2)/2;
                    Qk(2,4) = (X(6)*q_u*sin(X(5))*dt^3)/3 + (q_v*cos(X(5))*dt^2)/2;

                    Qk(3,1) = Qk(1,3);
                    Qk(3,2) = Qk(2,3);
                    Qk(3,3) = ((q_v*X(6)^2)/3 + (q_r*X(4)^2)/3)*dt^3 + q_u*dt;
                    Qk(3,4) = - (q_r*X(3)*X(4)*dt^3)/3 + ((X(6)*q_u)/2 - (X(6)*q_v)/2)*dt^2;
                    Qk(3,5) = -(dt^3*q_r*X(4))/3;
                    Qk(3,6) = -(dt^2*q_r*X(4))/2;

                    Qk(4,1) = Qk(1,4);
                    Qk(4,2) = Qk(2,4);
                    Qk(4,3) = Qk(3,4);
                    Qk(4,4) = ((q_u*X(6)^2)/3 + (q_r*X(3)^2)/3)*dt^3 + q_v*dt;
                    Qk(4,5) = (dt^3*q_r*X(3))/3;
                    Qk(4,6) = (dt^2*q_r*X(3))/2;

                    Qk(5,5) = (dt^3*q_r)/3;
                    Qk(5,6) = (dt^2*q_r)/2;

                    Qk(6,5) = Qk(5,6);
                    Qk(6,6) = dt*q_r;

                    Ut = [ zeros(8,1); 49 ];

                    % 시간 업데이트 (예측 단계)
                    xhat = srk1(@vehicle_dynamics_Airbearing_stochastic_debug, X, Ut, Qk, dt, params, dt);
                    P = F_non * P * F_non' + Qk;

                    pos_target = [2202.8; 1531.01; 0; 0];  % 목표물 위치 속도
                    att_target = [deg2rad(-90); deg2rad(0)];  % 목표물 자세 각속도
                    X_target = [ pos_target; att_target ];

                    dxm = xhat(1) - X_target(1);
                    dym = xhat(2) - X_target(2);

                    % 관측 모델 H 계산
                    H = zeros(3, 6);
                    range_miner = sqrt(dxm^2 + dym^2);

                    % 관측 행렬 H 구성
                    H(1,1) = dxm / range_miner;
                    H(1,2) = dym / range_miner;

                    H(2,1) = -dym / (dxm^2 + dym^2);
                    H(2,2) = dxm / (dxm^2 + dym^2);
                    H(2,5) = -1;

                    H(3,6) = 1;

                    % 예측 측정값 h 계산
                    h = measurement_model_RB(xhat, X_target);

                    % 혁신 계산
                    yhat = z - h;
                    yhat(2) = wrapToPi(yhat(2));
                    yhat(3) = wrapToPi(yhat(3));

                    % 칼만 이득 계산
                    S = H * P * H' + R;
                    K = P * H' / S;

                    % 상태 업데이트
                    X = xhat + K * yhat;
                    P = (eye(6) - K * H) * P * (eye(6) - K * H)' + K * R * K';  % Joseph form

                    previous_P = P;
                    X_Predict = xhat;

                    % 실시간 업데이트 (필요 시)
                    updateUI(double(global_translation), X(1), X(2), X(5));

                else
                    disp('No subjects data received.');
                    rawdata = previous_rawdata;
                    P = previous_P;
                end
            end
        end
    catch ME
        disp(['Error parsing data: ', ME.message]);
        rawdata = previous_rawdata;
        P = previous_P;
    end
end
