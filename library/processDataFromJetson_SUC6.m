function [ X, P, total_latency, interval, rawdata, X_Predict ] = processDataFromJetson_SUC6( client, X, Ut, params, dt, prevTime )
    persistent previous_att;
    persistent previous_Pos;
    persistent previous_rawdata;
    persistent previous_P;
    persistent filtered_rate;  % 각속도 필터링을 위한 변수 추가

    if isempty(filtered_rate)
        filtered_rate = 0;
    end

    total_latency = 0;
    interval = 0;
    rawdata_init = [ 716; -1291; 0; 0; deg2rad(0); 0 ];

    if isempty(previous_rawdata)
        rawdata = rawdata_init;
        previous_rawdata = rawdata;
    else
        rawdata = previous_rawdata;
    end

    if isempty(previous_P)
        P = params.EKF.P;
        previous_P = P;
    else
        P = previous_P;
    end

    X_Predict = X;
    aruco_data = struct();

    % 시스템 잡음 조정
    qs_u = 0.01;  % 선속도 잡음 감소
    qs_v = 0.01;
    qs_r = deg2rad(0.1);  % 각속도 잡음 크게 감소
    qs_x = 0.001;  % 위치 잡음 감소
    qs_y = 0.001;
    qs_psi = deg2rad(0.1);  % 자세각 잡음 감소

    Qs = diag([qs_x.^2, qs_y.^2, qs_u.^2, qs_v.^2, qs_psi.^2, qs_r.^2]);

    % 측정 잡음 조정
    Rq_rho = 0.05;  % 거리 측정 신뢰도 증가
    Rq_theta = deg2rad(5);  % 각도 측정 불확실성 증가
    Rq_rm = deg2rad(0.5);  % AHRS 측정 신뢰도 증가
    Rs = diag([Rq_rho.^2, Rq_theta.^2, Rq_rm.^2]);
    R_base = Rs;

    data = receiveDataFromJetson(client);

    if isempty(data)
        rawdata = previous_rawdata;
        P = previous_P;
        X_Predict = X;
        return;
    end

    try
        jsonLines = strsplit(char(data'), '\n');

        for i = 1:length(jsonLines)
            jsonData = strtrim(jsonLines{i});
            if isempty(jsonData)
                continue;
            end

            parsedData = jsondecode(jsonData);

            if isfield(parsedData, 'timestamp')
                timestamp_C = double(int64(posixtime(datetime('now'))));
                total_latency = timestamp_C - parsedData.timestamp;
                interval = toc(prevTime);

                % AHRS 데이터 처리 개선
                if isfield(parsedData, 'euler_rate')
                    z_AHRS = deg2rad(parsedData.euler_rate.yaw);
                    
                    % 각속도 필터링 (단순 저주파 필터)
                    alpha = 0.1;  % 필터링 계수 (0.1은 강한 필터링)
                    filtered_rate = (1-alpha) * filtered_rate + alpha * z_AHRS;
                    z_AHRS = filtered_rate;
                else
                    z_AHRS = 0;
                end

                % ArUco 데이터 처리
                if isfield(parsedData, 'aruco') && ~isempty(parsedData.aruco)
                    aruco_ids = fieldnames(parsedData.aruco);
                    marker_id = aruco_ids{1};
                    aruco_data = parsedData.aruco.(marker_id);
                    
                    % 측정값 이상치 제거
                    if abs(aruco_data.distance) > 5 || abs(deg2rad(aruco_data.euler_angles.pitch)) > pi/2
                        continue;
                    end
                    
                    z_AR = [aruco_data.distance; deg2rad(aruco_data.euler_angles.pitch)];
                else
                    continue;  % ArUco 데이터가 없으면 업데이트 스킵
                end

                % 예측 단계
                xhat = srk1(@vehicle_dynamics_Airbearing_stochastic_debug, X, Ut, Qs, dt, params, dt);
                F = get_state_transition_matrix(X, dt);  % 상태 전이 행렬 계산
                P = F * P * F' + Qs;

                % ArUco 마커 위치
                marker_pos = [aruco_data.translation(1); aruco_data.translation(3)];
                
                % 측정 행렬 계산
                H = compute_measurement_matrix(xhat, marker_pos);
                
                % 측정값 예측
                h = measurement_model_RB(xhat, marker_pos);
                
                % 실제 측정값
                z = [z_AR; z_AHRS];

                % 혁신 순서 계산 및 각도 정규화
                yhat = z - h;
                yhat(2:3) = wrapToPi(yhat(2:3));

                % 칼만 게인 계산
                S = H * P * H' + R_base;
                K = P * H' / S;

                % 상태 업데이트
                X = xhat + K * yhat;
                X(5) = wrapToPi(X(5));  % 자세각 정규화
                X(6) = wrapToPi(X(6));  % 각속도 정규화

                % 각속도 제한
                X(6) = min(max(X(6), -pi/2), pi/2);  % 각속도 제한

                % 공분산 업데이트
                P = (eye(6) - K * H) * P;
                P = (P + P')/2;  % 대칭성 보장

                previous_P = P;
                previous_rawdata = rawdata;
                X_Predict = xhat;

                % UI 업데이트 (필요 시)
                updateUI(double(global_translation), X(1), X(2), X(5));
            end
        end
    catch ME
        disp(['Error: ', ME.message]);
        rawdata = previous_rawdata;
        P = previous_P;
        X_Predict = X;
    end
end

% 상태 전이 행렬 계산 함수
function F = get_state_transition_matrix(X, dt)
    F = [
        1, 0, dt*cos(X(5)), -dt*sin(X(5)), dt*(-X(3)*sin(X(5))-X(4)*cos(X(5))), 0;
        0, 1, dt*sin(X(5)), dt*cos(X(5)), dt*(X(3)*cos(X(5))-X(4)*sin(X(5))), 0;
        0, 0, 1, -dt*X(6), 0, -dt*X(4);
        0, 0, dt*X(6), 1, 0, dt*X(3);
        0, 0, 0, 0, 1, dt;
        0, 0, 0, 0, 0, 1
    ];
end

% 측정 행렬 계산 함수
function H = compute_measurement_matrix(x, X_target)
    dxm = x(1) - X_target(1);
    dym = x(2) - X_target(2);
    range_miner = sqrt(dxm^2 + dym^2);
    
    H = zeros(3,6);
    
    % Range measurement
    H(1,1) = dxm/range_miner;
    H(1,2) = dym/range_miner;
    
    % Bearing measurement
    H(2,1) = -dym/(dxm^2 + dym^2);
    H(2,2) = dxm/(dxm^2 + dym^2);
    H(2,5) = -1;
    
    % Angular rate measurement
    H(3,6) = 1;
end