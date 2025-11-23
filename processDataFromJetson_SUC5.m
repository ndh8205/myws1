function [ X, P, total_latency, interval, rawdata, X_Predict ] = processDataFromJetson_SUC5( client, X, Ut, params, dt, prevTime )

    persistent previous_att;
    persistent previous_Pos;
    persistent previous_rawdata;
    persistent previous_P;

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

    qs_u = 10;
    qs_v = 10;
    qs_r = deg2rad(10);
    qs_x = 0;
    qs_y = 0;
    qs_psi = deg2rad(0);

    Qs = diag( [ qs_x.^2, qs_y.^2, qs_u.^2, qs_v.^2, qs_psi.^2, qs_r.^2 ] );
    q_x = Qs(1,1);
    q_y = Qs(2,2);
    q_u = Qs(3,3);
    q_v = Qs(4,4);
    q_psi = Qs(5,5);
    q_r = Qs(6,6);

    Rq_rho = 20;
    Rq_theta = deg2rad(20);
    Rq_rm = deg2rad(1);
    Rs = diag( [ Rq_rho.^2, Rq_theta.^2, Rq_rm.^2 ] );
    R_base = Rs;

    Iz = params.Airbearing.I_Z;
    D = params.Airbearing.D_ref;
    M = params.Airbearing.m;
    T = params.Airbearing.Thrust;

    data = receiveDataFromJetson(client);

    if isempty(data)
        rawdata = previous_rawdata;
        P = previous_P;
        X_Predict = X;
        return;
    end

    try
        jsonLines = strsplit( char( data' ), '\n' );

        for i = 1 : length( jsonLines )
            jsonData = strtrim( jsonLines{i} );

            if isempty( jsonData )
                continue;
            end

            parsedData = jsondecode( jsonData );

            if isfield(parsedData, 'timestamp')
                timestamp_C = double(int64(posixtime(datetime('now'))));
                total_latency = timestamp_C - parsedData.timestamp;

                interval = toc( prevTime );

                % ArUco 데이터 처리
                if isfield(parsedData, 'aruco') && ~isempty(parsedData.aruco)
                    aruco_ids = fieldnames(parsedData.aruco);
                    marker_id = aruco_ids{1};
                    aruco_data.translation = parsedData.aruco.(marker_id).translation;
                    aruco_data.euler_angles = parsedData.aruco.(marker_id).euler_angles;
                    aruco_data.distance = parsedData.aruco.(marker_id).distance;
                    z_AR = [ aruco_data.distance; deg2rad(aruco_data.euler_angles.pitch) ];
                else
                    z_AR = [];
                end

                % AHRS 데이터 처리
                if isfield(parsedData, 'euler_rate')  % 'ahrs_data' 제거
                    % euler_rate field가 roll, pitch, yaw 필드를 가진 구조체로 들어옴
                    euler_rate = [
                        parsedData.euler_rate.roll;    % roll_rate
                        parsedData.euler_rate.pitch;   % pitch_rate
                        parsedData.euler_rate.yaw      % yaw_rate
                    ];
                    euler_rate = deg2rad(euler_rate);  % degree to radian
                    z_AHRS = euler_rate(3);           % yaw_rate만 사용
                    disp(z_AHRS)
                else
                    z_AHRS = [];
                    disp('no ahrs')
                end

                % Vicon 데이터 처리
                if isfield(parsedData, 'subjects') && ~isempty(parsedData.subjects)
                    subject = parsedData.subjects(1);
                    segment = subject.segments(1);

                    global_translation = segment.global_translation{1};
                    global_rotation = segment.global_rotation{1};

                    rawdata = [global_translation(1); global_translation(2); global_translation(3);
                             global_rotation(1); global_rotation(2); global_rotation(3)];

                    if isempty( previous_Pos )
                        previous_Pos = zeros( 2, 1 );
                        previous_att = 0;
                    end

                    Vel_now = ( rawdata( 1:2 ) - previous_Pos ) / dt;
                    rate_now = ( rawdata( 6 ) - previous_att ) / dt;

                    previous_Pos = rawdata( 1:2 );
                    previous_att = rawdata( 6 );

                    F = [
                        1, 0, dt*cos(X(5)), -dt*sin(X(5)), dt * ( -X(3)*sin(X(5)) - X(4)*cos(X(5)) ), 0;
                        0, 1, dt*sin(X(5)),  dt*cos(X(5)), dt * (  X(3)*cos(X(5)) - X(4)*sin(X(5)) ), 0;
                        0, 0, 1, -dt*X(6), 0, -dt*X(4);
                        0, 0, dt*X(6), 1, 0, dt*X(3);
                        0, 0, 0, 0, 1, dt;
                        0, 0, 0, 0, 0, 1
                    ];

                    % 프로세스 노이즈 공분산 행렬 Qk 계산
                    Qk = zeros(6,6);
                    Qk(1,1) = dt^3*((q_v*sin(X(5))^2)/3 - (q_u*(sin(X(5))^2 - 1))/3);
                    Qk(1,2) = (dt^3*sin(2*X(5))*(q_u - q_v))/6;
                    Qk(1,3) = (X(6)*q_v*sin(X(5))*dt^3)/3 + (q_u*cos(X(5))*dt^2)/2;
                    Qk(1,4) = (X(6)*q_u*cos(X(5))*dt^3)/3 - (q_v*sin(X(5))*dt^2)/2;
                    Qk(1,5) = 0;
                    Qk(1,6) = 0;

                    Qk(2,1) = Qk(1,2);
                    Qk(2,2) = dt^3*((q_u*sin(X(5))^2)/3 - (q_v*(sin(X(5))^2 - 1))/3);
                    Qk(2,3) = - (X(6)*q_v*cos(X(5))*dt^3)/3 + (q_u*sin(X(5))*dt^2)/2;
                    Qk(2,4) = (X(6)*q_u*sin(X(5))*dt^3)/3 + (q_v*cos(X(5))*dt^2)/2;
                    Qk(2,5) = 0;
                    Qk(2,6) = 0;

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

                    Qk(5,1) = Qk(1,5);
                    Qk(5,2) = Qk(2,5);
                    Qk(5,3) = Qk(3,5);
                    Qk(5,4) = Qk(4,5);
                    Qk(5,5) = (dt^3*q_r)/3;
                    Qk(5,6) = (dt^2*q_r)/2;

                    Qk(6,1) = Qk(1,6);
                    Qk(6,2) = Qk(2,6);
                    Qk(6,3) = Qk(3,6);
                    Qk(6,4) = Qk(4,6);
                    Qk(6,5) = Qk(5,6);
                    Qk(6,6) = dt*q_r;

                    Ut = zeros(9,1);

                    % 상태 예측
                    xhat = srk1(@vehicle_dynamics_Airbearing_stochastic_debug, X, Ut, Qk, dt, params, dt);
                    P = F * P * F' + Qk;

                    % 측정 모델
                    dxm = xhat(1) - aruco_data.translation(1);
                    dym = xhat(2) - aruco_data.translation(3);
                    range_miner = sqrt( dxm^2 + dym^2 );

                    H = zeros(3,6);
                    H(1,1) = dxm / range_miner;
                    H(1,2) = dym / range_miner;

                    H(2,1) = -dym / ( dxm^2 + dym^2 );
                    H(2,2) = dxm / ( dxm^2 + dym^2 );
                    H(2,5) = -1;

                    H(3,6) = 1;

                    % 기본 measurement model 적용
                    z = [ aruco_data.distance; deg2rad(aruco_data.euler_angles.pitch); 0 ];  % 임시로 0 추가
                    h = measurement_model_RB_modified(xhat, [aruco_data.translation(1); aruco_data.translation(3)]);
                    
                    % AHRS 데이터가 있을 때 yaw rate 업데이트
                    if ~isempty(z_AHRS)
                        z(3) = z_AHRS;  % 세 번째 요소를 AHRS 데이터로 교체
                    end
                    
                    yhat = z - h;
                    yhat(2) = wrapToPi(yhat(2));
                    yhat(3) = wrapToPi(yhat(3));

                    S = H * P * H' + R_base;
                    K = P * H' / S;

                    X = xhat + K * (z - h);
                    X(5) = wrapToPi(X(5));
                    X(6) = wrapToPi(X(6));

                    P = (eye(6) - K * H) * P * (eye(6) - K * H)' + K * R_base * K';
                    P = (P + P')/2;

                    previous_P = P;
                    previous_rawdata = rawdata;
                    X_Predict = xhat;

                    % UI 업데이트 (필요 시)
                    updateUI(double(global_translation), X(1), X(2), X(5));

                end % Vicon 데이터 처리 종료
            end % timestamp 존재 여부 확인 종료
        end % for 루프 종료
    catch ME
        disp(['Error parsing data: ', ME.message]);
        rawdata = previous_rawdata;
        P = previous_P;
        X_Predict = X;
    end % try-catch 블록 종료

end % 함수 종료

function h = measurement_model_RB_modified(x, cam_pos)
    % 카메라 좌표계에서의 거리와 각도 계산
    rho = sqrt(cam_pos(1)^2 + cam_pos(2)^2);
    theta = atan2(cam_pos(1), cam_pos(2));
    
    % 측정 벡터 구성
    h = [rho; theta; x(6)];
end
