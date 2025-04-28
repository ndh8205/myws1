function Qmekf = calcQmekfMatrix(X, dt, params)
    % (1) 상태 파싱
    px_hat = X(1);  
    py_hat = X(2);  
    pz_hat = X(3);
    u_hat  = X(4);  
    v_hat  = X(5);  
    w_hat  = X(6);
    qw_hat = X(7);  
    qx_hat = X(8);
    qy_hat = X(9);  
    qz_hat = X(10);
    wx_hat = X(11);
    wy_hat = X(12);
    wz_hat = X(13);

    % (2) 외력 등
    Fa_x=0; Fa_y=0; Fa_z=0;
    Ft_x=0; Ft_y=0; Ft_z=0;
    Fr_x=0; Fr_y=0; Fr_z=0;
    Fn_x=0; Fn_y=0; Fn_z=0;

    % (3) 파라미터
    m  = params.vehicle.m_D;
    g  = params.environment.g(3);
    Jx = params.vehicle.J_X;
    Jy = params.vehicle.J_Y;
    Jz = params.vehicle.J_Z;

    Ma_x=0; Ma_y=0; Ma_z=0;
    Mr_x=0; Mr_y=0; Mr_z=0;
    Mn_x=0; Mn_y=0; Mn_z=0;

    Cpx=0;

    % (4) 잡음 스케일
    s_px=0;
    s_py=0;
    s_pz=0;
    s_u=1500.01;
    s_v=1500.01;
    s_w=1500.01;
    s_thx=0;
    s_thy=0;
    s_thz=0;
    s_wx_=deg2rad(500.01);
    s_wy_=deg2rad(500.01);
    s_wz_=deg2rad(500.01);

    % (5) 실제 인자 53개를 그대로 맞춰서 호출
    Qmekf = calcQmekf_auto( ...
        px_hat, py_hat, pz_hat, ...     % 1..3
        u_hat, v_hat, w_hat, ...        % 4..6
        qw_hat, qx_hat, qy_hat, qz_hat, ... % 7..10
        wx_hat, wy_hat, wz_hat, ...     % 11..13
        Fa_x, Fa_y, Fa_z, ...           % 14..16
        Ft_x, Ft_y, Ft_z, ...           % 17..19
        Fr_x, Fr_y, Fr_z, ...           % 20..22
        Fn_x, Fn_y, Fn_z, ...           % 23..25
        m, g, ...                       % 26..27
        Ma_x, Ma_y, Ma_z, ...           % 28..30
        Mr_x, Mr_y, Mr_z, ...           % 31..33
        Mn_x, Mn_y, Mn_z, ...           % 34..36
        Cpx, ...                        % 37
        Jx, Jy, Jz, ...                 % 38..40
        dt, ...                         % 41
        s_px, s_py, s_pz, ...           % 42..44
        s_u, s_v, s_w, ...              % 45..47
        s_thx, s_thy, s_thz, ...        % 48..50
        s_wx_, s_wy_, s_wz_ ...         % 51..53
    );
end
