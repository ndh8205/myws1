function [ X, P, total_latency, interval, rawdata, X_Predict ] = PDFJ_UKF_ver1( client, X, Ut, params, dt, prevTime )

    persistent previous_att;
    persistent previous_Pos;
    persistent previous_rawdata;
    persistent previous_P;

    % 기본값 설정
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
    Q = params.UKF.Q;  % UKF를 위한 프로세스 노이즈
    R = params.UKF.R;  % UKF를 위한 측정 노이즈

    % UKF 파라미터 설정
    n = length(X);
    alpha = params.UKF.alpha;
    beta = params.UKF.beta;
    kappa = params.UKF.kappa;
    lambda = alpha^2 * (n + kappa) - n;

    % 가중치 계산
    Wm = [lambda / (n + lambda), repmat(1 / (2*(n + lambda)), 1, 2*n)];
    Wc = Wm;
    Wc(1) = Wc(1) + (1 - alpha^2 + beta);

    Iz = params.Airbearing.I_Z;
    D = params.Airbearing.D_ref;
    M = params.Airbearing.m;
    T = params.Airbearing.Thrust;
    
    % 데이터 수신
    data = receiveDataFromJetson(client);

    if isempty(data)
        rawdata = previous_rawdata;
        P = previous_P;
        X_Predict = [];
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
                         
                    % UKF 예측 단계

                    % 시그마 포인트 생성
                    L = chol( ( n + lambda ) * P, 'lower' );
                    X_sigma = [ X , X + L, X - L ];

                    X_sigma_pred = zeros(size(X_sigma));

                    for j = 1:size(X_sigma, 2)
                        X_sigma_pred(:,j) = rk4(@vehicle_dynamics_Airbearing, X_sigma(:,j), Ut, dt, params);
                    end
                    
                    % 예측된 평균과 공분산 계산
                    X_pred = X_sigma_pred * Wm';
                    P_pred = Q;

                    for j = 1:size(X_sigma_pred, 2)
                        P_pred = P_pred + Wc(j) * (X_sigma_pred(:,j) - X_pred) * (X_sigma_pred(:,j) - X_pred)';
                    end

                    % UKF 업데이트 단계

                    % 측정 모델에 따른 시그마 포인트 생성
                    % 여기서는 측정이 상태의 첫 6개와 직접 관련이 있다고 가정
                    Z_sigma = X_sigma_pred(1:6,:);  
                    
                    % 측정 예측
                    z_pred = Z_sigma * Wm';
                    
                    % 혁신 공분산 계산
                    Pzz = R;
                    for j = 1:size(Z_sigma, 2)
                        Pzz = Pzz + Wc(j) * (Z_sigma(:,j) - z_pred) * (Z_sigma(:,j) - z_pred)';
                    end
                    
                    % 상태와 측정 간의 교차 공분산 계산
                    Pxz = zeros(n, size(rawdata,1));
                    for j = 1:size(X_sigma_pred, 2)
                        Pxz = Pxz + Wc(j) * (X_sigma_pred(:,j) - X_pred) * (Z_sigma(:,j) - z_pred)';
                    end
                    
                    % 칼만 이득 계산
                    K = Pxz / Pzz;
                    
                    % 상태 및 공분산 업데이트
                    X = X_pred + K * (rawdata - z_pred);
                    P = P_pred - K * Pzz * K';
                    
                    previous_P = P;

                    % 예측된 상태 저장
                    X_Predict = X_pred;

                    % 필터링된 데이터
                    posX = X(1);
                    posY = X(2);
                    rotZ = X(5);

                    % 실시간 플롯 업데이트
                    updateUI( double( global_translation ), posX, posY, rotZ );

                else
                    disp('subjects 데이터가 수신되지 않았습니다.');
                    rawdata = previous_rawdata;
                    P = previous_P;
                    X_Predict = [];
                end
            end
        end

    catch ME
        disp(['데이터 파싱 오류: ', ME.message]);
        rawdata = previous_rawdata;
        P = previous_P;
        X_Predict = [];
    end
end
