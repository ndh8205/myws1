function [ X, P, total_latency, interval, rawdata_VICON_SAT, X_Predict ] = PDFJ_EKF_ver1( client, X, Ut, params, dt, prevTime )

    persistent previous_att;
    persistent previous_Pos;
    persistent previous_rawdata_VICON_SAT;
    persistent previous_rawdata_VICON_MAS;
    persistent previous_P;

    % Default Value set
    total_latency = 0;
    interval = 0;
    rawdata_init = [ 416; -1291; 0; 0; deg2rad(0); 0 ];
    P_init = params.EKF.P;
    X_Predict = X;
    aruco_data = struct();
    % disp(dt)

    if isempty(previous_rawdata_VICON_SAT)

        rawdata_VICON_SAT = rawdata_init;
        rawdata_VICON_MAS = rawdata_init;
        previous_rawdata_VICON_SAT = rawdata_VICON_SAT;
        previous_rawdata_VICON_MAS = rawdata_VICON_MAS;

    end

    if isempty(previous_P)

        P = P_init;
        previous_P = P;

    end

    P = previous_P;
    Q = params.EKF.Q .* 10;
    R = params.EKF.Rm .* 100;

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

        rawdata_VICON_SAT = previous_rawdata_VICON_SAT;
        rawdata_VICON_MAS = previous_rawdata_VICON_MAS;
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

                if isfield(parsedData, 'translation') && isfield(parsedData, 'rotation_vector') && isfield(parsedData, 'distance')
                    
                    aruco_data.translation = parsedData.translation;
                    aruco_data.rotation_euler = parsedData.rotation_vector;
                    aruco_data.distance = ( parsedData.distance + 20 ) * 1000 ;

                    z_theta = atan2( aruco_data.translation(2), aruco_data.translation(1) );
                    z_theta = wrapToPi(z_theta);

                    dist_AR = sqrt(aruco_data.translation(2)^2 +  aruco_data.translation(1)^2);

                    z_AR = [ dist_AR ; z_theta ];
                end
                
                if isfield(parsedData, 'euler_angle') && isfield(parsedData, 'quaternion')
            
                    euler_rates = parsedData.euler_angle;

                    AHRS_R = deg2rad( euler_rates.yaw );

                    z_ARS = [ z_AR; AHRS_R ];
            
                end
                
                % subjects 정보 가져오기
                if isfield(parsedData, 'subjects') && ~isempty(parsedData.subjects)
                    
                    subject1 = parsedData.subjects(1);  % 첫 번째 subject
                    subject2 = parsedData.subjects(2);  % 첫 번째 subject

                    segment1 = subject1.segments(1);  % 첫 번째 segment
                    segment2 = subject2.segments(1);  % 첫 번째 segment


                    global_translation1 = segment1.global_translation{1};  % 위치 정보
                    global_rotation1 = segment1.global_rotation{1};  % 회전 정보

                    global_translation2 = segment2.global_translation{1};  % 위치 정보
                    global_rotation2 = segment2.global_rotation{1};  % 회전 정보

                    

                    rawdata_VICON_SAT = [global_translation1(1); global_translation1(2); global_translation1(3);
                         global_rotation1(1); global_rotation1(2); global_rotation1(3)];

                    rawdata_VICON_MAS = [global_translation2(1); global_translation2(2); global_translation2(3);
                         global_rotation2(1); global_rotation2(2); global_rotation2(3)];

                    % disp('Master : ')
                    % disp(rawdata_VICON_MAS)
                    % disp('SAT : ')
                    % disp(rawdata_VICON_SAT)

                    % norm()

                    if isempty( previous_Pos )
    
                        previous_Pos = zeros( 2, 1 );
                        previous_att = 0;
            
                    end

                    Vel_now = ( rawdata_VICON_SAT( 1:2 ) - previous_Pos ) / dt;
                    rate_now = ( rawdata_VICON_SAT( 6 ) - previous_att ) / dt;

                    previous_Pos = rawdata_VICON_SAT( 1:2 );
                    previous_att = rawdata_VICON_SAT( 6 );

                    z_sat_VICON = [ rawdata_VICON_SAT(1); rawdata_VICON_SAT(2); Vel_now; rawdata_VICON_SAT(6); rate_now ];
                    z_mas_VICON = [ rawdata_VICON_MAS(1); rawdata_VICON_MAS(2); rawdata_VICON_MAS(6) ];

                    previous_rawdata_VICON_SAT = rawdata_VICON_SAT;
                             
                    % Given values

                    F =  [
                            1, 0, dt * cos(X(5)), -dt*sin(X(5)), dt * ( -X(3) * sin(X(5)) - X(4) * cos( X(5) ) ),        0;
                            0, 1, dt * sin(X(5)),  dt*cos(X(5)), dt * (  X(3) * cos(X(5)) - X(4) * sin( X(5) ) ),        0;
                            0, 0,              1,      -dt*X(6),                                               0, -dt*X(4);
                            0, 0,      dt * X(6),             1,                                               0,  dt*X(3);
                            0, 0,              0,             0,                                               1,       dt;
                            0, 0,              0,             0,                                               0,        1
                         
                          ];
                    
                    B = [
                            0,          0,           0,          0,           0,          0,           0,          0       0;
                            0,          0,           0,          0,           0,          0,           0,          0       0;
                            0,       -T/M,        -T/M,          0,           0,        T/M,         T/M,          0       0;
                         -T/M,          0,           0,        T/M,         T/M,          0,           0,       -T/M       0;
                            0,          0,           0,          0,           0,          0,           0,          0       0;
                    -(D*T)/Iz,   (D*T)/Iz,   -(D*T)/Iz,   (D*T)/Iz,   -(D*T)/Iz,   (D*T)/Iz,   -(D*T)/Iz,   (D*T)/Iz     1/Iz
                         ] .* dt;


                    
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
                                        
                    xhat = srk1( @vehicle_dynamics_Airbearing_stochastic, X, Ut, Qk, dt, params, dt );

                    P = F * P * F' + Qk ;

                    dxm = xhat(1) - z_mas_VICON(1);
                    dym = xhat(2) - z_mas_VICON(2);
                
                    % Inertial frame range
                    range_miner = sqrt( dxm^2 + dym^2 );
                
                    % H - Range-bearing observation model (2d)
                    H = zeros(3, 6);
                    
                    % First row
                    H(1,1) = dxm / range_miner;
                    H(1,2) = dym / range_miner;
                    
                    % Second row
                    H(2,1) = -dym / ( dxm^2 + dym^2 );
                    H(2,2) = dxm / ( dxm^2 + dym^2 );
                    H(2,5) = -1;
                
                    % Third row
                    H(3,6) = 1;

                    h = measurement_model_RB( xhat, z_mas_VICON );

                    % Compute innovation
                    yhat = z_ARS - h;

                    % Kalman gain
                    S = H * P * H' + R;
                    K = P * H' * inv(S) ;

                    % Update
                    X = xhat + K * yhat;
                    P = ( eye(6) - K * H ) * P * ( eye( 6 ) - K * H )' + K * R * K'; % Joseph form

                    previous_P = P;

                    X_Predict = xhat;

                    % Real time plot update
                    updateUI( double( rawdata_VICON_SAT ), X(1), X(2), X(5) );         

                else

                    disp('No subjects data received.');
                    rawdata_VICON_SAT = previous_rawdata_VICON_SAT;
                    P = previous_P;

                end

            end

        end

    catch ME

        disp(['Error parsing data: ', ME.message]);
        rawdata_VICON_SAT = previous_rawdata_VICON_SAT;
        P = previous_P;

    end

end
