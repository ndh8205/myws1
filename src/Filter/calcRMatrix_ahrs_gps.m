function R = calcRMatrix_ahrs_gps(params)

    Rq_lat = 0.2;
    Rq_long = 0.2;
    Rq_alt = 0.3;

    Rq_p = deg2rad(0.01);
    Rq_q = deg2rad(0.01);
    Rq_r = deg2rad(0.01);

    Rq_ax = 0.01;
    Rq_ay = 0.01;
    Rq_az = 0.01;

    Rq_mx = 0.01;
    Rq_my = 0.01;
    Rq_mz = 0.01;

    R = diag( [ Rq_lat.^2, Rq_long.^2, Rq_alt.^2, Rq_p.^2, Rq_q.^2, Rq_r.^2 ...
        Rq_ax.^2, Rq_ay.^2, Rq_az.^2, Rq_mx.^2, Rq_my.^2, Rq_mz.^2,] );

end