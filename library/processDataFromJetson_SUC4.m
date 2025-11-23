function [ X, P, total_latency, interval, rawdata, X_Predict ] = processDataFromJetson_SUC4( client, X, Ut, params, dt, prevTime )

    persistent previous_att;
    persistent previous_Pos;
    persistent previous_rawdata;
    persistent previous_P;

    % Default Value set
    total_latency = 0;
    interval = 0;
    rawdata_init = [ 716; -1291; 0; 0; deg2rad(0); 0 ];
    P_init = params.EKF.P;
    X_Predict = X;
    aruco_data = struct();

    if isempty(previous_rawdata)
        rawdata = rawdata_init;
        previous_rawdata = rawdata;
    end

    if isempty(previous_P)
        P = P_init;
        previous_P = P;
    end

    P = previous_P;

    % 프로세스 노이즈 및 측정 노이즈 초기화
    qs_u = 20.095;
    qs_v = 20.095;
    qs_r = deg2rad(20.095);
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

    Rq_rho = 5;
    Rq_theta = deg2rad(5);
    Rq_rm = deg2rad(1);
    Rs = diag( [ Rq_rho.^2, Rq_theta.^2, Rq_rm.^2 ] );
    R = Rs;

    Iz = params.Airbearing.I_Z;
    D = params.Airbearing.D_ref;
    M = params.Airbearing.m;
    T = params.Airbearing.Thrust;

    % Recieve Data
    data = receiveDataFromJetson(client);

    if isempty(data)
        rawdata = previous_rawdata;
        P = previous_P;
        return;
    end

    try 
        % 수신된 데이터를 줄 단위로 분할
        jsonLines = strsplit( char( data' ), '\n' );

        for i = 1 : length( jsonLines )
            jsonData = strtrim( jsonLines{i} );  % 데이터 문자열로 변환 및 공백 제거

            if isempty( jsonData )
                continue;
            end

            parsedData = jsondecode( jsonData );

            % 타임스탬프 및 데이터 처리
            if isfield(parsedData, 'timestamp')
                timestamp_C = double(int64(posixtime(datetime('now'))));
                total_latency = timestamp_C - parsedData.timestamp;

                % 수신 주기 측정
                interval = toc( prevTime );

                % ArUco 데이터 처리
                if isfield(parsedData, 'aruco') && ~isempty(parsedData.aruco)
                    aruco_ids = fieldnames(parsedData.aruco);
                    % 첫 번째 마커만 사용 (필요에 따라 수정 가능)
                    marker_id = aruco_ids{1};
                    aruco_data.translation = parsedData.aruco.(marker_id).translation;
                    aruco_data.euler_angles = parsedData.aruco.(marker_id).euler_angles;
                    aruco_data.distance = parsedData.aruco.(marker_id).distance;

                    % 측정 벡터 생성
                    z_AR = [ aruco_data.distance; deg2rad(aruco_data.euler_angles.pitch) ];
                else
                    z_AR = [];
                end

                % AHRS 데이터 처리 (필요에 따라 추가)

                 % AHRS 데이터 처리
                if isfield(parsedData, 'ahrs_data') && isfield(parsedData.ahrs_data, 'euler_rate')
                    % AHRS로부터 얻은 yaw 값을 사용
                    ahrs_yaw = deg2rad(parsedData.ahrs_data.euler_rate.yaw);
                    % 측정 벡터에 추가
                    z_AHRS = ahrs_yaw;
                else
                    z_AHRS = [];
                end

                % Vicon 데이터 처리
                if isfield(parsedData, 'subjects') && ~isempty(parsedData.subjects)
                    subject = parsedData.subjects(1);  % 첫 번째 subject
                    segment = subject.segments(1);  % 첫 번째 segment

                    global_translation = segment.global_translation{1};  % 위치 정보
                    global_rotation = segment.global_rotation{1};  % 회전 정보

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

                    % 상태 전이 행렬(F) 계산
                    F = [
                        1, 0, dt*cos(X(5)), -dt*sin(X(5)), dt * ( -X(3)*sin(X(5)) - X(4)*cos(X(5)) ), 0;
                        0, 1, dt*sin(X(5)),  dt*cos(X(5)), dt * (  X(3)*cos(X(5)) - X(4)*sin(X(5)) ), 0;
                        0, 0, 1, -dt*X(6), 0, -dt*X(4);
                        0, 0, dt*X(6), 1, 0, dt*X(3);
                        0, 0, 0, 0, 1, dt;
                        0, 0, 0, 0, 0, 1
                    ];

                    % 프로세스 노이즈 공분산 행렬(Qk) 계산
                    % Define all elements
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
                    xhat = srk1( @vehicle_dynamics_Airbearing_stochastic_debug, X, Ut, Qk, dt, params, dt );
                    P = F * P * F' + Qk;

                    % 측정 모델 (시뮬레이션 코드의 Range-Bearing 모델 적용)
                    dxm = xhat(1) - aruco_data.translation(1);
                    dym = xhat(2) - aruco_data.translation(2);
                    range_miner = sqrt( dxm^2 + dym^2 );

                    H = zeros(3,6);
                    H(1,1) = dxm / range_miner;
                    H(1,2) = dym / range_miner;
                    H(2,1) = -dym / ( dxm^2 + dym^2 );
                    H(2,2) = dxm / ( dxm^2 + dym^2 );
                    H(2,5) = -1;
                    H(3,6) = 1;

                    % 측정 예측
                    h = measurement_model_RB( xhat, [aruco_data.translation(1); aruco_data.translation(2)] );

                    % 혁신 계산
                    z = [ aruco_data.distance; deg2rad(aruco_data.euler_angles.yaw); rate_now ];
                    yhat = z - h;
                    yhat(2) = wrapToPi(yhat(2));
                    yhat(3) = wrapToPi(yhat(3));

                    % 칼만 이득 계산
                    S = H * P * H' + R;
                    K = P * H' / S;

                    % 상태 업데이트
                    X = xhat + K * yhat;
                    X(5) = wrapToPi( X(5) );
                    X(6) = wrapToPi( X(6) );

                    % 공분산 업데이트
                    P = ( eye(6) - K * H ) * P * ( eye(6) - K * H )' + K * R * K'; % Joseph form

                    previous_P = P;
                    X_Predict = xhat;

                    % UI 업데이트 (필요에 따라)
                    updateUI( double( global_translation ), X(1), X(2), X(5) );         

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
