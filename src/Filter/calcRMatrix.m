function R = calcRMatrix(params)

    Rq_rat = 0.2;
    Rq_long = 0.2;
    Rq_alt = 0.3;

    Rq_p = deg2rad(0.01);
    Rq_q = deg2rad(0.01);
    Rq_r = deg2rad(0.01);

    R = diag( [ Rq_rat.^2, Rq_long.^2, Rq_alt.^2, Rq_p.^2, Rq_q.^2, Rq_r.^2 ] );

end