function [ X, P, total_latency, interval, rawdata1, X_Predict ] = PDFJ_Ver2( client, X, Ut, params, dt, prevTime )

    persistent previous_att1 previous_att2;
    persistent previous_Pos1 previous_Pos2;
    persistent previous_rawdata1 previous_rawdata2;
    persistent previous_P;

     % Default Value set
    total_latency = 0;
    interval = 0;
    rawdata_init = [ 2074; -307; 0; 0; 0; 0 ];
    P_init = params.EKF.P;
    X_Predict = X;
    aruco_data = struct();

    if isempty(previous_rawdata1) || isempty(previous_rawdata2)

        rawdata1 = rawdata_init;
        rawdata2 = rawdata_init;
        previous_rawdata1 = rawdata1;
        previous_rawdata2 = rawdata2;

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

        rawdata1 = previous_rawdata1;
        rawdata2 = previous_rawdata2;

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
                if isfield( parsedData, 'translation') && isfield(parsedData, 'rotation_vector') && isfield(parsedData, 'distance' )

                    aruco_data.translation = parsedData.translation;
                    aruco_data.rotation_vector = parsedData.rotation_vector;
                    aruco_data.distance = parsedData.distance * 1000;

                    [camera_position, camera_rotation] = getCameraPose(aruco_data.translation, aruco_data.rotation_vector);

                    disp("Camera Position:");
                    disp(camera_position);
                    disp("Camera Rotation (Euler angles in degrees):");
                    disp(rad2deg(camera_rotation));
                    % disp("translation : ")
                    % disp( aruco_data.translation )
                    % 
                    % disp("rotation_E : ")
                    % disp( rad2deg(rotation_euler) )
                    % disp("rotation_V : ")
                    % disp( rad2deg(aruco_data.rotation_vector) )

                    % z_rho3d = norm( [ aruco_data.translation(1); aruco_data.translation(2); aruco_data.translation(3) ] );
                    % z_rho2d = aruco_data.distance * cos(aruco_data.rotation_euler(2));

                    % z_AR = [ aruco_data.distance; rotation_euler(2) ];
                    % disp(z_rho2d)

                end
                
               % Process subjects information
                if isfield(parsedData, 'subjects') && length(parsedData.subjects) >= 2

                    % Process first object
                    subject1 = parsedData.subjects(1);
                    segment1 = subject1.segments(1);
                    global_translation1 = segment1.global_translation{1};
                    global_rotation1 = segment1.global_rotation{1};
                    rawdata1 = [global_translation1(1); global_translation1(2); global_translation1(3);
                                global_rotation1(1); global_rotation1(2); global_rotation1(3)];

                    % Process second object
                    subject2 = parsedData.subjects(2);
                    segment2 = subject2.segments(1);
                    global_translation2 = segment2.global_translation{1};
                    global_rotation2 = segment2.global_rotation{1};
                    rawdata2 = [global_translation2(1); global_translation2(2); global_translation2(3);
                                global_rotation2(1); global_rotation2(2); global_rotation2(3)];

                    if isempty(previous_Pos1) || isempty(previous_Pos2)

                        previous_Pos1 = zeros(2, 1);
                        previous_Pos2 = zeros(2, 1);
                        previous_att1 = 0;
                        previous_att2 = 0;

                    end

                    % Calculate velocities and rates for both objects
                    Vel_now1 = (rawdata1(1:2) - previous_Pos1) / dt;
                    rate_now1 = (rawdata1(6) - previous_att1) / dt;
                    Vel_now2 = (rawdata2(1:2) - previous_Pos2) / dt;
                    rate_now2 = (rawdata2(6) - previous_att2) / dt;

                    previous_Pos1 = rawdata1(1:2);
                    previous_Pos2 = rawdata2(1:2);
                    previous_att1 = rawdata1(6);
                    previous_att2 = rawdata2(6);

                    z1 = [rawdata1(1); rawdata1(2); Vel_now1; rawdata1(6); rate_now1];
                    z2 = [rawdata2(1); rawdata2(2); Vel_now2; rawdata2(6); rate_now2];

                    % For now, we'll use z1 for the filter. You might want to combine z1 and z2 or use them separately depending on your needs.
                    z = z2;

                    previous_rawdata1 = rawdata1;
                    previous_rawdata2 = rawdata2;

                else

                    disp('Not enough subjects data received.');
                    rawdata1 = previous_rawdata1;
                    rawdata2 = previous_rawdata2;
                    P = previous_P;

                    return;

                end
            end
        end

    catch ME

        disp(['Error parsing data: ', ME.message]);
        rawdata1 = previous_rawdata1;
        rawdata2 = previous_rawdata2;

        P = previous_P;

        return;

    end

    % % Given values
    % F =  [
    %         1, 0, dt * cos(X(5)), -dt*sin(X(5)), dt * ( -X(3) * sin(X(5)) - X(4) * cos( X(5) ) ),        0;
    %         0, 1, dt * sin(X(5)),  dt*cos(X(5)), dt * (  X(3) * cos(X(5)) - X(4) * sin( X(5) ) ),        0;
    %         0, 0,              1,      -dt*X(6),                                               0, -dt*X(4);
    %         0, 0,      dt * X(6),             1,                                               0,  dt*X(3);
    %         0, 0,              0,             0,                                               1,       dt;
    %         0, 0,              0,             0,                                               0,        1
    %      ];
    % 
    % B = [
    %         0,          0,           0,          0,           0,          0,           0,          0       0;
    %         0,          0,           0,          0,           0,          0,           0,          0       0;
    %         0,       -T/M,        -T/M,          0,           0,        T/M,         T/M,          0       0;
    %      -T/M,          0,           0,        T/M,         T/M,          0,           0,       -T/M       0;
    %         0,          0,           0,          0,           0,          0,           0,          0       0;
    % -(D*T)/Iz,   (D*T)/Iz,   -(D*T)/Iz,   (D*T)/Iz,   -(D*T)/Iz,   (D*T)/Iz,   -(D*T)/Iz,   (D*T)/Iz     1/Iz
    %      ] .* dt;
    % 
    % Qs = diag( [ Q(1,1).^2, Q(2,2).^2, Q(3,3).^2, Q(4,4).^2, Q(5,5).^2, Q(6,6).^2 ] );
    % 
    % q_x = Qs(1,1);
    % q_y = Qs(2,2);
    % q_u = Qs(3,3);
    % q_v = Qs(4,4);
    % q_psi = Qs(5,5);
    % q_r = Qs(6,6);
    % 
    % % Define Qk matrix
    % Qk = zeros(6,6);
    % 
    % % Define all elements
    % Qk(1,1) = dt^3 * ( ( q_v * sin( X(5) )^2 ) / 3 - ( q_u * ( sin( X(5) )^2 - 1) ) / 3 );
    % Qk(1,2) = ( dt^3 * sin( 2 * X(5) ) * ( q_u - q_v ) ) / 6;
    % Qk(1,3) = ( X(6) * q_v * sin( X(5) ) * dt^3 ) / 3 + ( q_u * cos( X(5) ) * dt^2 ) / 2;
    % Qk(1,4) = ( X(6) * q_u * cos( X(5) ) * dt^3 ) / 3 - ( q_v * sin( X(5) ) * dt^2 ) / 2;
    % Qk(1,5) = 0;
    % Qk(1,6) = 0;
    % 
    % Qk(2,1) = Qk(1,2);
    % Qk(2,2) = dt^3 * ( ( q_u * sin( X(5) )^2) / 3 - ( q_v * ( sin( X(5) )^2 - 1) ) / 3 );
    % Qk(2,3) = -( X(6) * q_v * cos( X(5) ) * dt^3 ) / 3 + ( q_u * sin( X(5) ) * dt^2 ) / 2;
    % Qk(2,4) =  ( X(6) * q_u * sin( X(5) ) * dt^3 ) / 3 + ( q_v * cos( X(5) ) * dt^2) / 2;
    % Qk(2,5) = 0;
    % Qk(2,6) = 0;
    % 
    % Qk(3,1) = Qk(1,3);
    % Qk(3,2) = Qk(2,3);
    % Qk(3,3) = ( ( q_v * X(6)^2 ) / 3 + ( q_r * X(4)^2) / 3 ) * dt^3 + q_u * dt;
    % Qk(3,4) = - ( q_r * X(3)* X(4) * dt^3 ) / 3 + ( ( X(6) * q_u ) / 2 - ( X(6) * q_v ) / 2 ) * dt^2;
    % Qk(3,5) = -( dt^3 * q_r * X(4) ) / 3;
    % Qk(3,6) = -( dt^2 * q_r * X(4) ) / 2;
    % 
    % Qk(4,1) = Qk(1,4);
    % Qk(4,2) = Qk(2,4);
    % Qk(4,3) = Qk(3,4);
    % Qk(4,4) = ( ( q_u * X(6)^2 ) / 3 + ( q_r * X(3)^2) / 3) * dt^3 + q_v * dt;
    % Qk(4,5) = ( dt^3 * q_r * X(3) ) / 3;
    % Qk(4,6) = ( dt^2 * q_r * X(3) ) / 2;
    % 
    % Qk(5,1) = Qk(1,5);
    % Qk(5,2) = Qk(2,5);
    % Qk(5,3) = Qk(3,5);
    % Qk(5,4) = Qk(4,5);
    % Qk(5,5) = (dt^3*q_r)/3;
    % Qk(5,6) = (dt^2*q_r)/2;
    % 
    % Qk(6,1) = Qk(1,6);
    % Qk(6,2) = Qk(2,6);
    % Qk(6,3) = Qk(3,6);
    % Qk(6,4) = Qk(4,6);
    % Qk(6,5) = Qk(5,6);
    % Qk(6,6) = dt*q_r;
    % 
    % xhat = F * X + B * Ut;
    % 
    % P = F * P * F' + Qk ;
    % 
    % % Measurement model ( Vicon Full)
    % H = [ 1, 0, 0, 0, 0, 0 ;
    %       0, 1, 0, 0, 0, 0 ;
    %       0, 0, 1, 0, 0, 0 ;
    %       0, 0, 0, 1, 0, 0 ;
    %       0, 0, 0, 0, 1, 0 ;
    %       0, 0, 0, 0, 0, 1 ];
    % 
    % h = H * xhat;
    % 
    % % Compute innovation
    % yhat = z - h;
    % 
    % % Kalman gain
    % S = H * P * H' + R;
    % K = P * H' * inv(S) ;
    % 
    % % Update
    % X = xhat + K * yhat;
    % P = ( eye(6) - K * H ) * P * ( eye( 6 ) - K * H )' + K * R * K'; % Joseph form
    % 
    % previous_P = P;
    % 
    % X_Predict = xhat;

    % Real time plot update

    rho = z_AR(1);
    the = deg2rad(-9);

    psi = deg2rad(-1.7);

    X_ta = z2(1);
    Y_ta = z2(2);

    alpha = atan2( ( Y_ta ), ( X_ta ) );

    RB2I = [ cos(psi), -sin(psi); sin(psi), cos(psi)];
    % RI2B = RB2I';
    
    % rho_cart_B = [ rho * cos(the); rho * sin(the)];
    rho_cart_B = [ rho * sin(the); rho * cos(the)];

    rho_cart_I = RB2I * rho_cart_B;
    
    X_sat_I = X_ta - rho_cart_I(1);
    Y_sat_I = Y_ta - rho_cart_I(2);

    psi = deg2rad(-1.7);  

    X = [ X_sat_I; Y_sat_I; z1(3:4); psi; z(6) ];

    % updateUI( double( global_translation2 ), X(1), X(2), X(5) );
    updateUI( double( global_translation2 ), X_sat_I, Y_sat_I, psi );         

end


function [camera_position, camera_rotation] = getCameraPose(marker_translation, marker_rotation_vector)
    % 마커의 회전 행렬 계산
    R_marker = rotationVectorToMatrix( marker_rotation_vector );
    
    % 카메라의 회전 행렬 (마커에 대한 카메라의 회전)
    R_camera = R_marker';
    
    % 카메라의 위치 (마커 좌표계에서)
    camera_position = -R_camera * marker_translation';
    
    % 카메라의 회전을 오일러 각도로 변환
    camera_rotation = rotationMatrixToEuler(R_camera);
end

function R = rotationVectorToMatrix(rv)
    theta = norm(rv);
    if theta < eps
        R = eye(3);
    else
        k = rv / theta;
        K = [0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
        R = eye(3) + sin(theta)*K + (1-cos(theta))*K^2;
    end
end

function euler = rotationMatrixToEuler(R)
    % ZYX 순서의 오일러 각도로 변환
    if abs(R(3,1)) ~= 1
        pitch = -asin(R(3,1));
        roll = atan2(R(3,2) / cos(pitch), R(3,3) / cos(pitch));
        yaw = atan2(R(2,1) / cos(pitch), R(1,1) / cos(pitch));
    else
        yaw = 0;
        if R(3,1) == -1
            pitch = pi/2;
            roll = yaw + atan2(R(1,2), R(1,3));
        else
            pitch = -pi/2;
            roll = -yaw + atan2(-R(1,2), -R(1,3));
        end
    end
    euler = [roll; pitch; yaw];
end