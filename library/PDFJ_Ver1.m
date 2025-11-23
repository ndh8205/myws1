function [ X, P, total_latency, interval, rawdata, X_Predict ] = PDFJ_Ver1( client, X, Ut, params, dt, prevTime )

    persistent previous_att;
    persistent previous_Pos;
    persistent previous_rawdata;
    persistent previous_P;

    % Default Value set
    total_latency = 0;
    interval = 0;
    rawdata_init = [ 716; -1291; 0; 0; -deg2rad(180); 0 ];
    P_init = params.EKF.P;
    X_Predict = X;
    aruco_data = struct();
    disp(dt)

    if isempty(previous_rawdata)
        rawdata = rawdata_init;
        previous_rawdata = rawdata;
    end

    if isempty(previous_P)
        P = P_init;
        previous_P = P;
    end

    P = previous_P;
    Q = params.EKF.Q .* 1;
    R = params.EKF.R .* 1;

    Iz = params.Airbearing.I_Z;
    D = params.Airbearing.D_ref;
    M = params.Airbearing.m;
    T = params.Airbearing.Thrust;
    
    % Receive Data
    data = receiveDataFromJetson(client);

    if isempty(data)
        rawdata = previous_rawdata;
        P = previous_P;
        return;
    end
    
    try 
        % Split received data into lines
        jsonLines = strsplit( char( data' ), '\n' );

        for i = 1 : length( jsonLines )
            jsonData = strtrim( jsonLines{i} );  % Convert to string and remove whitespace

            if isempty( jsonData )
                continue;
            end

            parsedData = jsondecode( jsonData );

            % Process timestamp and data
            if isfield(parsedData, 'timestamp')
                timestamp_C = double(int64(posixtime(datetime('now'))));
                total_latency = timestamp_C - parsedData.timestamp;

                % Measure reception interval
                interval = toc( prevTime );

                % Process Aruco data if available
                if isfield(parsedData, 'translation') && isfield(parsedData, 'rotation_euler') && isfield(parsedData, 'distance')
                    aruco_data.translation = parsedData.translation;
                    aruco_data.rotation_euler = parsedData.rotation_euler;
                    aruco_data.distance = parsedData.distance;
    
                    z_aurco = [aruco_data.translation(1); aruco_data.translation(2); aruco_data.translation(3);
                         aruco_data.rotation_euler(1); aruco_data.rotation_euler(2); aruco_data.rotation_euler(3)];

                    z_AR = [ aruco_data.distance; rad2deg(aruco_data.rotation_euler(3))];
                    disp(z_AR)
                end
                
                % Process subjects information
                if isfield(parsedData, 'subjects') && ~isempty(parsedData.subjects)
                    subject = parsedData.subjects(1);  % First subject
                    segment = subject.segments(1);  % First segment

                    global_translation = segment.global_translation{1};  % Position information
                    global_rotation = segment.global_rotation{1};  % Rotation information

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

                    z = [ rawdata(1); rawdata(2); Vel_now; rawdata(6); rate_now ];

                    previous_rawdata = rawdata;
                else
                    disp('No subjects data received.');
                    rawdata = previous_rawdata;
                    P = previous_P;
                    return;
                end
            end
        end

    catch ME
        disp(['Error parsing data: ', ME.message]);
        rawdata = previous_rawdata;
        P = previous_P;
        return;
    end

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

    Qs = diag( [ Q(1,1).^2, Q(2,2).^2, Q(3,3).^2, Q(4,4).^2, Q(5,5).^2, Q(6,6).^2 ] );

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
    
    xhat = F * X + B * Ut;

    P = F * P * F' + Qk ;

    % Measurement model ( Vicon Full)
    H = [ 1, 0, 0, 0, 0, 0 ;
          0, 1, 0, 0, 0, 0 ;
          0, 0, 1, 0, 0, 0 ;
          0, 0, 0, 1, 0, 0 ;
          0, 0, 0, 0, 1, 0 ;
          0, 0, 0, 0, 0, 1 ];

    h = H * xhat;

    % Compute innovation
    yhat = z - h;

    % Kalman gain
    S = H * P * H' + R;
    K = P * H' * inv(S) ;

    % Update
    X = xhat + K * yhat;
    P = ( eye(6) - K * H ) * P * ( eye( 6 ) - K * H )' + K * R * K'; % Joseph form

    previous_P = P;

    X_Predict = xhat;

    % Real time plot update
    updateUI( double( global_translation ), X(1), X(2), X(5) );         
end