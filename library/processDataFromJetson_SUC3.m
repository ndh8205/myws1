function [ X, P, total_latency, interval, rawdata, X_Predict ] = processDataFromJetson_SUC3( client, X, Ut, params, dt, prevTime )

    persistent previous_att;
    persistent previous_Pos;
    persistent previous_rawdata;
    persistent previous_P;

    % Default Value set
    total_latency = 0;
    interval = 0;
    rawdata_init = [ 0; 0; 0; 0; deg2rad(0); 0 ];
    P_init = params.EKF.P;
    X_Predict = X;
    aruco_data = struct();
    % disp(dt)



    if isempty(previous_rawdata)

        rawdata = rawdata_init;
        previous_rawdata = rawdata;

    end

    if isempty(previous_P)

        P = P_init;
        previous_P = P;

    end


    P = previous_P;
    Q = params.UKF.Q .* 100;
    R = params.EKF.Rv .* 1;

    qs_x = Q(1,1);
    qs_y = Q(2,2);
    qs_u = Q(3,3);
    qs_v = Q(4,4);
    qs_psi = Q(5,5);
    qs_r = Q(6,6);

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

                if isfield(parsedData, 'translation') && isfield(parsedData, 'rotation_euler') && isfield(parsedData, 'distance')
                    
                    aruco_data.translation = parsedData.translation;
                    aruco_data.rotation_euler = parsedData.rotation_euler;
                    aruco_data.distance = parsedData.distance;
    
                    z_aurco = [aruco_data.translation(1); aruco_data.translation(2); aruco_data.translation(3);
                         aruco_data.rotation_euler(1); aruco_data.rotation_euler(2); aruco_data.rotation_euler(3)];

                    % z_aurco_dist = norm( [z_aurco(1), z_aurco(2), z_aurco(3)] );
                    z_AR = [ aruco_data.distance; rad2deg(aruco_data.rotation_euler(3))];
                    disp(z_AR)
                end
                
                % subjects 정보 가져오기
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

                    % z = [ rawdata(1); rawdata(2); Vel_now; rawdata(6); rate_now ];
                    z = [ rawdata(1); rawdata(2); rawdata(6); ];

                    previous_rawdata = rawdata;
                             
                    % Given values
                    
                    Qs = diag( [ qs_x.^2, qs_y.^2, qs_u.^2, qs_v.^2, qs_psi.^2, qs_r.^2 ] );
                    
                    q_x = Qs(1,1);
                    q_y = Qs(2,2);
                    q_u = Qs(3,3);
                    q_v = Qs(4,4);
                    q_psi = Qs(5,5);
                    q_r = Qs(6,6);
                
                    % Define Qk matrix
                    Qk = zeros(6,6);
                    
                    % Define all elements
                    Qk(1,1) = dt^3 * ( ( q_v * sin( X(5) )^2 ) / 3 - ( q_u * ( sin( X(5) )^2 - 1) ) / 3 );
                    Qk(1,2) = ( dt^3 * sin( 2 * X(5) ) * ( q_u - q_v ) ) / 6;
                    Qk(1,3) = ( X(6) * q_v * sin( X(5) ) * dt^3 ) / 3 + ( q_u * cos( X(5) ) * dt^2 ) / 2;
                    Qk(1,4) = ( X(6) * q_u * cos( X(5) ) * dt^3 ) / 3 - ( q_v * sin( X(5) ) * dt^2 ) / 2;
                    Qk(1,5) = 0;
                    Qk(1,6) = 0;
                    
                    Qk(2,1) = Qk(1,2);
                    Qk(2,2) = dt^3 * ( ( q_u * sin( X(5) )^2) / 3 - ( q_v * ( sin( X(5) )^2 - 1) ) / 3 );
                    Qk(2,3) = -( X(6) * q_v * cos( X(5) ) * dt^3 ) / 3 + ( q_u * sin( X(5) ) * dt^2 ) / 2;
                    Qk(2,4) =  ( X(6) * q_u * sin( X(5) ) * dt^3 ) / 3 + ( q_v * cos( X(5) ) * dt^2) / 2;
                    Qk(2,5) = 0;
                    Qk(2,6) = 0;
                    
                    Qk(3,1) = Qk(1,3);
                    Qk(3,2) = Qk(2,3);
                    Qk(3,3) = ( ( q_v * X(6)^2 ) / 3 + ( q_r * X(4)^2) / 3 ) * dt^3 + q_u * dt;
                    Qk(3,4) = - ( q_r * X(3)* X(4) * dt^3 ) / 3 + ( ( X(6) * q_u ) / 2 - ( X(6) * q_v ) / 2 ) * dt^2;
                    Qk(3,5) = -( dt^3 * q_r * X(4) ) / 3;
                    Qk(3,6) = -( dt^2 * q_r * X(4) ) / 2;
                    
                    Qk(4,1) = Qk(1,4);
                    Qk(4,2) = Qk(2,4);
                    Qk(4,3) = Qk(3,4);
                    Qk(4,4) = ( ( q_u * X(6)^2 ) / 3 + ( q_r * X(3)^2) / 3) * dt^3 + q_v * dt;
                    Qk(4,5) = ( dt^3 * q_r * X(3) ) / 3;
                    Qk(4,6) = ( dt^2 * q_r * X(3) ) / 2;
                    
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

                    Ut = [ zeros(8,1); 49 ];    

                    n = length(X);
                    m = length(z);
                
                    % Unscented Kalman Filter parameter set
                    alpha = 1e-3;
                    beta = 2;
                    kappa = 0;
                
                    lambda = alpha^2 * (n + kappa) - n;
                
                    % weight
                    Wm = zeros(2*n + 1,1); % mean
                    Wc = zeros(2*n + 1,1); % covariance
                
                    Wm(1) = lambda / (n + lambda);
                    Wc(1) = Wm(1) + (1 - alpha^2 + beta);
                
                    for i = 2:2*n+1
                        Wm(i) = 1 / (2*(n + lambda));
                        Wc(i) = Wm(i);
                    end
                
                
                    % sigma point generate
                    sigma_pts = zeros(n, 2*n + 1);
                    sigma_pts(:,1) = X;
                    sqrtP = chol((n + lambda)*P, 'lower');
                
                    for i = 1:n
                        sigma_pts(:, i+1) = X + sqrtP(:,i);
                        sigma_pts(:, n + i + 1) = X - sqrtP(:,i);
                    end

                    % sigma point predict
                    sigma_pts_pred = zeros(n, 2*n + 1);
                
                    for i2 = 1:2*n+1
                        x = sigma_pts(:,i2);
                        xdot = vehicle_dynamics_Airbearing_stochastic( x, Ut, zeros(3,1), dt, params );
                        x_pred = x + xdot * dt;
                        sigma_pts_pred(:,i2) = x_pred;
                    end
                
                    % predict state
                    xhat = zeros(n,1);
                    for i2 = 1:2*n+1
                        xhat = xhat + Wm(i2) * sigma_pts_pred(:,i2);
                    end
                
                    % predict covariance
                    P_hat = zeros(n,n);
                    for i2 = 1:2*n+1
                        dx = sigma_pts_pred(:,i2) - xhat;
                        P_hat = P_hat + Wc(i2) * (dx) * (dx)';
                    end
                
                    % P hat
                    P_hat = P_hat + Qk;
                
                    % measurement predict
                    Z_sigma = zeros(m, 2*n + 1);
                    for i2 = 1:2*n+1
                        x = sigma_pts_pred(:,i2);
                        % range-bearing measurement_model update
                        Z_sigma(:,i2) = measurement_model_VI(x, 0);
                    end
                
                    % predict measurement
                    zhat = zeros(m,1);
                    for i2 = 1:2*n+1
                        zhat = zhat + Wm(i2) * Z_sigma(:,i2);
                    end
                
                    % measurment covariance
                
                    P_zz = zeros(m,m);
                    for i2 = 1:2*n+1
                        dz = Z_sigma(:,i2) - zhat;
                        P_zz = P_zz + Wc(i2) * dz * dz';
                    end
                
                    P_zz = P_zz + R;
                
                    % state & measurement covariance
                    P_xz = zeros(n,m);
                    for i2 = 1:2*n+1
                        dx = sigma_pts_pred(:,i2) - xhat;
                        dz = Z_sigma(:,i2) - zhat;
                        P_xz = P_xz + Wc(i2) * dx * dz';
                    end
                
                    % kalman gain
                    K = P_xz / P_zz;
                
                    % state update
                    yhat = z - zhat;
                    yhat(2) = wrapToPi(yhat(2));
                    yhat(3) = wrapToPi(yhat(3));
                
                    X = xhat + K * yhat;
                    X(5) = wrapToPi(X(5));
                    
                    % covariance update
                    P = P_hat - K * P_zz * K';
                  
                    previous_P = P;

                    X_Predict = xhat;

                    % Real time plot update
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
