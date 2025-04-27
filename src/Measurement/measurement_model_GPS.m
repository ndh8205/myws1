function [ gps_pos_meas, gps_pos_true ] = measurement_model_GPS(X, R, params)
    %--------------------------------------------------------------------------
    % measurement_model_GPS
    % 입력:
    %   X: 상태벡터
    %   R: 센서 노이즈 공분산 (9x9), diag구조 [GPS(3) gyro(3), accel(3), mag(3)]
    %   params: 구조체
    %       params.refLat, params.refLon, params.refAlt (기준점 위도·경도·고도)
    %       params.magYear (예: 2024.5)
    %       ...
    %
    % 출력:
    %   z_gps_meas (3x1): 자이로+가속도+마그넷 (노이즈 포함)
    %   z_gps_true (3x1): 자이로+가속도+마그넷 (이상적,노이즈 없음)
    %--------------------------------------------------------------------------

    %-----------------------------
    % 1) 상태 변수 추출
    %-----------------------------

    posNED    = X(1:3);     % NED(m)

    %-----------------------------
    % 2) 센서 노이즈 (std) 추출
    %-----------------------------
    Nu = sqrt(diag(R));   % noise matrix to vector
    Nu_gps = Nu(1:3);   % gps noise

    % 실제 무작위 노이즈 샘플링
    gps_noise = Nu_gps .* randn(3,1);

    %-----------------------------
    % 3) NED -> (lat, lon, alt) 변환 및 wrldmagm 사용
    %-----------------------------

    lat0 = params.environment.latitude;   % deg
    lon0 = params.environment.longitude;   % deg
    alt0 = params.environment.altitude;   % m (기준 고도)

    % wgs84 타원체
    wgs84 = wgs84Ellipsoid('meters');

    % ned2geodetic(north,east,down, lat0,lon0,h0, ellipsoid)
    north_m_true = posNED(1);
    east_m_true  = posNED(2);
    down_m_true  = posNED(3);

    [ lat_deg, lon_deg, alt_m ] = ned2geodetic(north_m_true, east_m_true, down_m_true, ...
                                               lat0, lon0, alt0, wgs84);

    gps_pos_true = [ lat_deg; lon_deg; alt_m/1000 ];

    %-----------------------------
    % 4) 측정값 계산
    %-----------------------------

    north_m_meas = posNED(1) + gps_noise(1);
    east_m_meas  = posNED(2) + gps_noise(2);
    down_m_meas  = posNED(3) + gps_noise(3);

    [ lat_deg_m, lon_deg_m, alt_m_m ] = ned2geodetic(north_m_meas, east_m_meas, down_m_meas, ...
                                               lat0, lon0, alt0, wgs84);

    gps_pos_meas = [lat_deg_m; lon_deg_m; alt_m_m/1000]; % 실제 측정 (noise)

end
