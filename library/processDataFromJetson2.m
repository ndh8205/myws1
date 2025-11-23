function [ X, P, total_latency, interval, rawdata ] = processDataFromJetson2( client, X, params, dt, U_T , prevTime )

    persistent previous_att;
    persistent previous_Pos;

    % 기본값 설정
    total_latency = 0;
    interval = 0;
    rawdata = [ 0, 0, 0, 0, 0, 0, 0, 0, 0 ]';

    P = params.EKF.P;
    
    % 데이터 수신
    data = receiveDataFromJetson(client);

    if isempty(data)
        return;
    end
    
    try
        % 수신된 데이터를 줄 단위로 분할
        jsonLines = strsplit(char(data'), '\n');

        for i = 1:length(jsonLines)

            jsonData = strtrim(jsonLines{i});  % 데이터 문자열로 변환 및 공백 제거

            if isempty(jsonData)
                continue;
            end

            parsedData = jsondecode(jsonData);
            disp(['Received data: ', jsonData]);


            % 타임스탬프 및 데이터 처리
            if isfield(parsedData, 'timestamp_A') && isfield(parsedData, 'timestamp_B')
                timestamp_A = double(parsedData.timestamp_A);
                timestamp_B = double(parsedData.timestamp_B);
                timestamp_C = double(int64(posixtime(datetime('now')) * 1000)); % 밀리초 단위로 변환
                latency_A_to_B = -timestamp_A + timestamp_B;
                latency_B_to_C = -timestamp_B + timestamp_C;
                total_latency = -timestamp_A + timestamp_C;

                % 수신 주기 측정
                interval = toc(prevTime) * 1000; 
                
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

                    rawdata =  [ z(1); z(2); Vel_now(1); Vel_now(2); z(6); rate_now ]; % x y u v psi r;

                    X = [ z(1); z(2); Vel_now(1); Vel_now(2); z(6); rate_now ]; % x y u v psi r

                    % 필터링된 위치 및 자세 정보
                    posX = X(1);
                    posY = X(2);
                    posZ = 0;

                    rotX = 0;
                    rotY = 0;
                    rotZ = X(5);

                    RI2B = GetDCM_Euler( rotZ, rotY, rotX );
                    RB2I = RI2B';

                    % 실시간 플롯 업데이트
                    updateUI(double(global_translation), posX, posY, posZ, rotZ, total_latency, interval, prevTime);
                else
                    disp('No subjects data received.');
                end
            end
        end
    catch ME
        disp(['Error parsing data: ', ME.message]);
    end
end