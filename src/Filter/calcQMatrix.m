
function Qk = calcQmekfMatrix(X, dt, params)

    u  = X(4);  v  = X(5);  w  = X(6);
    qw = X(7);  qx = X(8);  qy = X(9);  qz = X(10);
    wx = X(11); wy = X(12); wz = X(13);

    m  = params.vehicle.m_D;
    g  = params.environment.g(3);
    Jx = params.vehicle.J_X;
    Jy = params.vehicle.J_Y;
    Jz = params.vehicle.J_Z;

    s_x  = 0;
    s_y  = 0;
    s_z  = 0;
    s_u  = 100.01;
    s_v  = 100.01;
    s_w  = 100.01;
    s_qw = 0;
    s_qx = 0;
    s_qy = 0;
    s_qz = 0;
    s_wx = deg2rad(100.01);
    s_wy = deg2rad(100.01);
    s_wz = deg2rad(100.01);

    Qk = calcQk_auto(u,v,w, qw,qx,qy,qz, ...
                     wx,wy,wz, m,g,Jx,Jy,Jz, dt, ...
                     s_x^2, s_y^2, s_z^2, s_u^2, s_v^2, s_w^2, ...
                     s_qw^2, s_qx^2, s_qy^2, s_qz^2, s_wx^2, s_wy^2, s_wz^2);
end