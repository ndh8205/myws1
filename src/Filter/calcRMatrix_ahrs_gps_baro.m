function R = calcRMatrix_ahrs_gps_baro(params)

    Rq_n = 0.2;
    Rq_e = 0.2;
    Rq_d = 0.3;

    Rq_p = deg2rad(0.01);
    Rq_q = deg2rad(0.01);
    Rq_r = deg2rad(0.01);

    Rq_ax = 0.01;
    Rq_ay = 0.01;
    Rq_az = 0.01;

    Rq_mx = deg2rad(1);
    Rq_my = deg2rad(1);
    Rq_mz = deg2rad(1);

    Rq_baro = 0.2;

    R = diag( [ Rq_n.^2, Rq_e.^2, Rq_d.^2, Rq_p.^2, Rq_q.^2, Rq_r.^2 ...
        Rq_ax.^2, Rq_ay.^2, Rq_az.^2, Rq_mx.^2, Rq_my.^2, Rq_mz.^2, Rq_baro.^2] );

end