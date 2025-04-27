function h = measurement_model_HANul( xhat, params )

    lat_init = params.environment.latitude;   % [deg]
    lon_init = params.environment.longitude;  % [deg]
    alt_init = params.environment.altitude;   % [m]
    wgs84 = wgs84Ellipsoid('meters');

    [ z_ahrs_meas, z_ahrs_bias, z_ahrs_true ] = measurement_model_AHRS(xhat, zeros(13,13), params);
    [ gps_pos_meas, gps_pos_true ] = measurement_model_GPS(xhat, zeros(13,13), params);

    lat_deg = gps_pos_true(1);
    lon_deg = gps_pos_true(2);
    alt_km  = gps_pos_true(3);
    alt_m = alt_km * 1000;
    [north_m, east_m, down_m] = geodetic2ned(lat_deg, lon_deg, alt_m, lat_init, lon_init, alt_init, wgs84);
    gps_pos_NED_t = [ north_m ; east_m ; down_m ];

    [ z_baro_meas, z_baro_bias, z_baro_true ] = measurement_model_baro(xhat, zeros(13,13), params);

    h = [ z_ahrs_bias; gps_pos_NED_t; z_baro_bias];

end