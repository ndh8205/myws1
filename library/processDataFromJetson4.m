function [ X, P, total_latency, interval, rawdata, X_Predict ] = processDataFromJetson4( client, X, Ut, params, dt, prevTime )

    persistent previous_att;
    persistent previous_Pos;
    persistent previous_rawdata;
    persistent previous_P;

    % Default Value set
    total_latency = 0;
    interval = 0;
    rawdata_init = [ 0 0 0 0 0 0 ]';
    P_init = params.UKF.P;

    if isempty(previous_rawdata)
        rawdata = rawdata_init;
        previous_rawdata = rawdata;
    end

    if isempty(previous_P)
        P = P_init;
        previous_P = P;
    end

    P = previous_P;
    Q = params.UKF.Q;  % Changed from EKF to UKF
    R = params.UKF.R;  % Changed from EKF to UKF

    % UKF parameters
    n = length(X);
    alpha = params.UKF.alpha;
    beta = params.UKF.beta;
    kappa = params.UKF.kappa;
    lambda = alpha^2 * (n + kappa) - n;

    % Weights calculation
    Wm = [lambda / (n + lambda), repmat(1 / (2*(n + lambda)), 1, 2*n)];
    Wc = Wm;
    Wc(1) = Wc(1) + (1 - alpha^2 + beta);

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
            if isfield( parsedData, 'timestamp_A' ) && isfield( parsedData, 'timestamp_B' )
                timestamp_A = double( parsedData.timestamp_A );
                timestamp_B = double( parsedData.timestamp_B );
                timestamp_C = double( int64( posixtime( datetime( 'now' ) ) ) );
                total_latency = -timestamp_A + timestamp_C;

                % 수신 주기 측정
                interval = toc( prevTime );
                
                % subjects 정보 가져오기
                if isfield(parsedData, 'subjects') && ~isempty(parsedData.subjects)
                    
                    subject = parsedData.subjects(1);  % 첫 번째 subject
                    segment = subject.segments(1);  % 첫 번째 segment

                    global_translation = segment.global_translation{1};  % 위치 정보
                    global_rotation = segment.global_rotation{1};  % 회전 정보

                    z = [global_translation(1); global_translation(2); global_translation(3);
                         global_rotation(1); global_rotation(2); global_rotation(3)];

                    if isempty( previous_Pos )
                        previous_Pos = zeros( 2, 1 );
                        previous_att = 0;
                    end

                    Vel_now = ( z( 1:2 ) - previous_Pos ) / dt;
                    rate_now = ( z( 6 ) - previous_att ) / dt;

                    previous_Pos = z( 1:2 );
                    previous_att = z( 6 );

                    rawdata = [ z(1); z(2); Vel_now; z(6); rate_now ];
                    previous_rawdata = rawdata;
                             
                    % UKF Prediction Step

                    % Generate sigma points
                    L = chol( ( n + lambda ) * P, 'lower' );

                    X_sigma = [ X , X + L, X - L ];

                    X_sigma_pred = zeros(size(X_sigma));

                    for j = 1:size(X_sigma, 2)
                        X_sigma_pred(:,j) = rk4(@vehicle_dynamics_Airbearing, X_sigma(:,j), Ut, dt, params);
                    end
                    
                    % Calculate predicted mean and covariance
                    X_pred = X_sigma_pred * Wm';
                    P_pred = Q;

                    for j = 1:size(X_sigma_pred, 2)

                        P_pred = P_pred + Wc(j) * (X_sigma_pred(:,j) - X_pred) * (X_sigma_pred(:,j) - X_pred)';

                    end

                    % UKF Update Step

                    % Generate measurement sigma points
                    Z_sigma = X_sigma_pred(1:6,:);  % Assuming measurement is directly related to first 6 states
                    
                    % Calculate predicted measurement
                    z_pred = Z_sigma * Wm';
                    
                    % Calculate innovation covariance
                    Pzz = R;

                    for j = 1:size(Z_sigma, 2)

                        Pzz = Pzz + Wc(j) * (Z_sigma(:,j) - z_pred) * (Z_sigma(:,j) - z_pred)';

                    end
                    
                    % Calculate cross-covariance

                    Pxz = zeros(size(X_pred,1), size(rawdata,1));

                    for j = 1:size(X_sigma_pred, 2)

                        Pxz = Pxz + Wc(j) * (X_sigma_pred(:,j) - X_pred) * (Z_sigma(:,j) - z_pred)';

                    end
                    
                    % Kalman gain
                    K = Pxz / Pzz;
                    
                    % Update state and covariance
                    X = X_pred + K * (rawdata - z_pred);
                    P = P_pred - K * Pzz * K';

                    previous_P = P;

                    % filterd_data
                    posX = X(1);
                    posY = X(2);
                    rotZ = X(5);

                    % Real time plot update
                    updateUI( double( global_translation ), posX, posY, rotZ );

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