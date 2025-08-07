function Heskf = calcHeskfMatrix(X_23, dt, params)
%--------------------------------------------------------------------------
% calcHeskfMatrix (Error-state, 22차)
%
%  - "AHRS(9) + GPS(3) + Baro(1)" => 총 13차 측정 모델
%  - 오차 상태 차원: 22 (deltaX)
%    ==> H 행렬 크기: (13 x 22)
%
% [입력]
%   - X_23(23x1) : 공칭 상태
%       = [ p(3), v(3), q(4), w(3), b_gyro(3), b_acc(3), b_mag(3), b_baro(1) ]
%   - dt       : 샘플링 간격 (여기서는 사용 안할 수도 있음)
%   - params   : 구조체 (지자기 등 센서 관련 파라미터)
%
% [출력]
%   - Heskf(13x22)
%       = "에러 상태" 기반 관측모델 자코비안
%--------------------------------------------------------------------------
    %-----------------------------
    % (1) 상태 파싱 (공칭 상태, 23x1)
    %-----------------------------
    px_hat = X_23(1);
    py_hat = X_23(2);
    pz_hat = X_23(3);
    u_hat  = X_23(4);
    v_hat  = X_23(5);
    w_hat  = X_23(6);
    qw_hat = X_23(7);
    qx_hat = X_23(8);
    qy_hat = X_23(9);
    qz_hat = X_23(10);
    wx_hat = X_23(11);
    wy_hat = X_23(12);
    wz_hat = X_23(13);

    bgx_hat = X_23(14);
    bgy_hat = X_23(15);
    bgz_hat = X_23(16);
    bax_hat = X_23(17);
    bay_hat = X_23(18);
    baz_hat = X_23(19);
    bmx_hat = X_23(20);
    bmy_hat = X_23(21);
    bmz_hat = X_23(22);
    bbaro_hat = X_23(23);

    %-----------------------------
    % (2) 지자기 벡터 (NED) 계산
    %-----------------------------
    % posNED = [p_x, p_y, p_z] = (N, E, D)
    posNED = [px_hat; py_hat; pz_hat];

    lat0 = params.environment.latitude;    % [deg]
    lon0 = params.environment.longitude;   % [deg]
    alt0 = params.environment.altitude;    % [m]
    decYear = 2024.5;

    wgs84 = wgs84Ellipsoid('meters');
    [lat_deg, lon_deg, alt_m] = ned2geodetic(...
        posNED(1), posNED(2), posNED(3), ...
        lat0, lon0, alt0, wgs84);
    alt_km = alt_m / 1000;

    % wrldmagm => [N, E, D] in nT
    mag_xyz_nT = wrldmagm(alt_km, lat_deg, lon_deg, decYear);
    M_I_wmm = mag_xyz_nT * 1e-9;  % [N, E, D] in T
    Mx = M_I_wmm(1);
    My = M_I_wmm(2);
    Mz = M_I_wmm(3);

    %-----------------------------
    % (3) H행렬(13x22) 계산
    %   - "calcHeskf_22_error_auto.m" (심볼릭 자동생성) 호출
    %-----------------------------
    %   이 함수는 "오차상태 기반" 자코비안:
    %   H = d z / d deltaX (13x22), deltaX=0 근방
    Heskf = calcHeskf_22_error_auto( ...
       px_hat, py_hat, pz_hat, ...
       u_hat, v_hat, w_hat, ...
       qw_hat, qx_hat, qy_hat, qz_hat, ...
       wx_hat, wy_hat, wz_hat, ...
       bgx_hat, bgy_hat, bgz_hat, ...
       bax_hat, bay_hat, baz_hat, ...
       bmx_hat, bmy_hat, bmz_hat, ...
       bbaro_hat, ...
       Mx, My, Mz ...
    );

end
