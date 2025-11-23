function [ X, P, total_latency, interval, rawdata, X_Predict ] = processDataFromJetson_multi_1( client, X, Ut, params, dt, prevTime )

    persistent previous_att;
    persistent previous_Pos;
    persistent previous_rawdata;
    persistent previous_P;


    % Default Value set
    total_latency = 0;
    interval = 0;
    rawdata_init = [ 0 0 0 0 0 0 ]';
    P_init = params.EKF.P;
    X_Predict = X;

    if isempty(previous_rawdata)

        rawdata = rawdata_init;
        
        previous_rawdata = rawdata;

    end

    if isempty(previous_P)

        P = P_init;

        previous_P = P;

    end

    P = previous_P;
    Q = params.EKF.Q;
    R = params.EKF.R;

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
    
    new_data = struct('field1', [], 'field2', [], 'field3', []);

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

                % 새로운 데이터 필드 추출
                if isfield(parsedData, 'new_field1')
                    new_data.field1 = parsedData.new_field1;
                end
                if isfield(parsedData, 'new_field2')
                    new_data.field2 = parsedData.new_field2;
                end
                if isfield(parsedData, 'new_field3')
                    new_data.field3 = parsedData.new_field3;
                end
                
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
                                        
                    xhat = F * X + B * Ut;
                    % xhat = F * X;

                    P = F * P * F' + Qk ;


                    % Measurement model
                    H = [ 1, 0, 0, 0, 0, 0 ;
                          0, 1, 0, 0, 0, 0 ;
                          0, 0, 1, 0, 0, 0 ;
                          0, 0, 0, 1, 0, 0 ;
                          0, 0, 0, 0, 1, 0 ;
                          0, 0, 0, 0, 0, 1 ];

                    h = H * xhat;

                    % Compute innovation
                    y = rawdata - h;

                    % Kalman gain
                    S = H * P * H' + R;
                    K = P * H' * inv(S) ;

                    % Update
                    X = xhat + K * y;
                    P = ( eye(6) - K * H ) * P * ( eye( 6 ) - K * H )' + K * R * K'; % Joseph form
                    previous_P = P;

                    X_Predict = xhat;

                    % Real time plot update
                    updateUI( double( global_translation ), X(1), X(2), X(5) );         

                else

                    disp('No subjects data received.');
                    rawdata = previous_rawdata;
                    P = previous_P;
                    new_data = struct('field1', [], 'field2', [], 'field3', []);

                end

            end

        end

    catch ME

        disp(['Error parsing data: ', ME.message]);
        rawdata = previous_rawdata;
        P = previous_P;
        new_data = struct('field1', [], 'field2', [], 'field3', []);

    end

end
