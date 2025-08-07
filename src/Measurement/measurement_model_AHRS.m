function [ z_ahrs_meas, z_ahrs_bias, z_ahrs_true ] = measurement_model_AHRS(X, R, params)
    %--------------------------------------------------------------------------
    % measurement_model_AHRS
    %
    %   [ pos(3); velB(3); quat(4); omega(3); b_g(3); b_a(3); b_m(3) ]
    %   
    %   - pos(1:3) : NED 기준 위치 (m) -> [N, E, D]
    %   - velB(1:3): 바디축 선속도
    %   - quat(4)  : 자세(쿼터니언)
    %   - omega(3) : 바디축 각속도
    %   - b_g(3)   : 자이로 바이어스
    %   - b_a(3)   : 가속도 바이어스
    %   - b_m(3)   : 마그넷 바이어스
    %
    % 입력:
    %   X: 상태벡터
    %   R: 센서 노이즈 공분산 (9x9), diag구조 [GPS(3) gyro(3), accel(3), mag(3)]
    %   params: 구조체
    %       params.refLat, params.refLon, params.refAlt (기준점 위도·경도·고도)
    %       params.magYear (예: 2024.5)
    %       ...
    %
    % 출력:
    %   z_ahrs_meas (9x1): 자이로+가속도+마그넷 (모두 바이어스+노이즈 포함)
    %   z_ahrs_bias (9x1): 자이로+가속도+마그넷 (바이어스 포함, 노이즈 없음)
    %   z_ahrs_true (9x1): 자이로+가속도+마그넷 (이상적, 바이어스·노이즈 없음)
    %--------------------------------------------------------------------------

    %-----------------------------
    % 1) 상태 변수 추출
    %-----------------------------
    quat   = X(7:10);     % quaternion
    omegaB = X(11:13);    % (p, q, r) - body frame
    bg     = X(14:16);    % gyro bias
    ba     = X(17:19);    % acc bias
    bm     = X(20:22);    % mag bias

    %-----------------------------
    % 2) 센서 노이즈 (std) 추출
    %-----------------------------
    Nu = sqrt(diag(R));   % noise matrix to vector
    Nu_gyr = Nu(4:6);   % gyro noise
    Nu_acc = Nu(7:9);   % acc noise
    Nu_mag = Nu(10:12);   % mag noise

    % 실제 무작위 노이즈 샘플링
    gyr_noise = Nu_gyr .* randn(3,1);
    acc_noise = Nu_acc .* randn(3,1);
    mag_noise = Nu_mag .* randn(3,1);

    %-----------------------------
    % 3) NED -> (lat, lon, alt) 변환 및 wrldmagm 사용
    %-----------------------------
    posNED = X(1:3);   % N, E, D [m]

    lat0 = params.environment.latitude;   % deg
    lon0 = params.environment.longitude;   % deg
    alt0 = params.environment.altitude;   % m (기준 고도)
    decYear = 2024.5;

    % wgs84 타원체
    wgs84 = wgs84Ellipsoid('meters');

    % ned2geodetic(north,east,down, lat0,lon0,h0, ellipsoid)
    north_m = posNED(1);
    east_m  = posNED(2);
    down_m  = posNED(3);

    [ lat_deg, lon_deg, alt_m ] = ned2geodetic(north_m, east_m, down_m, ...
                                               lat0, lon0, alt0, wgs84);

    % wrldmagm은 alt_km 필요
    alt_km = alt_m / 1000;
    mag_xyz = wrldmagm(alt_km, lat_deg, lon_deg, decYear);

    % nT -> T 변환
    % (N, E, D) = (x_nT, y_nT, z_nT)
    M_I_wmm = mag_xyz * 1e-9;

    %-----------------------------
    % 4) 측정값 계산
    %-----------------------------
    % (A) 자이로
    gyro_me = omegaB + bg + gyr_noise;   % 실제 측정 (bias+noise)
    gyro_bi = omegaB + bg;              % 바이어스만
    gyro_tr = omegaB;                   % 이상적(바이어스/노이즈無)

    % (B) 가속도
    R_b2i = GetDCM_QUAT(quat);  % Body -> NED
    R_i2b = R_b2i';
    g_I   = [0; 0; -9.81];
    g_b   = R_i2b * g_I;        % Body에서 본 중력
    a_body_est = [0; 0; 0];     % vehicle_dynamics 결과 등 필요 시 대입

    accel_me = a_body_est - g_b + ba + acc_noise;
    accel_bi = a_body_est - g_b + ba;
    accel_tr = a_body_est - g_b;

    % (C) 마그넷
    % wrldmagm 결과(M_I_wmm)를 사용
    mag_me = R_i2b * M_I_wmm + bm + mag_noise;
    mag_bi = R_i2b * M_I_wmm + bm;
    mag_tr = R_i2b * M_I_wmm;

    %-----------------------------
    % 5) 최종 출력
    %-----------------------------
    z_ahrs_meas = [ gyro_me; accel_me; mag_me ];  % 노이즈+바이어스
    z_ahrs_bias = [ gyro_bi; accel_bi; mag_bi ];  % 바이어스만
    z_ahrs_true = [ gyro_tr; accel_tr; mag_tr ];  % 이상적
end
