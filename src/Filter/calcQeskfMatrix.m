function Qeskf = calcQeskfMatrix(X_23, dt, params)
% calcQeskf22Matrix : ESKF(22차)용 Q행렬(22x22)을 계산
%
% [입력]
%   - X_23(23x1) : [p(3), v(3), q(4), w(3), b_gyro(3), b_acc(3), b_mag(3), b_baro(1)]
%   - dt         : 샘플링 간격
%   - params     : 구조체 (질량, 관성, FOGM 타임상수, 잡음 스케일 등)
%
% [출력]
%   - Q_22(22x22) : 프로세스 잡음 공분산행렬

    %-----------------------------
    % (1) 상태 파싱
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

    % 바이어스 항
    b_gx_hat = X_23(14);
    b_gy_hat = X_23(15);
    b_gz_hat = X_23(16);
    b_ax_hat = X_23(17);
    b_ay_hat = X_23(18);
    b_az_hat = X_23(19);
    b_mx_hat = X_23(20);
    b_my_hat = X_23(21);
    b_mz_hat = X_23(22);
    b_baro_hat = X_23(23);

    %-----------------------------
    % (2) 파라미터, 모멘트, etc.
    %-----------------------------
    m  = params.vehicle.m_D;
    g  = params.environment.g(3);
    Jx = params.vehicle.J_X;
    Jy = params.vehicle.J_Y;
    Jz = params.vehicle.J_Z;

    Ma_x=0; Ma_y=0; Ma_z=0;  % 필요시
    Mr_x=0; Mr_y=0; Mr_z=0;
    Mn_x=0; Mn_y=0; Mn_z=0;
    Cpx=0;

    tau_gyro = params.bias.tau_gyro;
    tau_acc  = params.bias.tau_acc;
    tau_mag  = params.bias.tau_mag;
    tau_baro = params.bias.tau_baro;

    %-----------------------------
    % (3) 잡음 스케일 설정
    %     실제 센서 스펙/튜닝값 등을 할당
    %-----------------------------
    s_px  = params.Qnoise.s_px;
    s_py  = params.Qnoise.s_py;
    s_pz  = params.Qnoise.s_pz;

    s_u   = params.Qnoise.s_u;
    s_v   = params.Qnoise.s_v;
    s_w   = params.Qnoise.s_w;

    s_thx = params.Qnoise.s_thx;
    s_thy = params.Qnoise.s_thy;
    s_thz = params.Qnoise.s_thz;

    s_wx_ = params.Qnoise.s_wx;
    s_wy_ = params.Qnoise.s_wy;
    s_wz_ = params.Qnoise.s_wz;

    % --- 추가로 10개 바이어스 잡음 ---
    s_dbgx   = params.Qnoise.s_dbgx;
    s_dbgy   = params.Qnoise.s_dbgy;
    s_dbgz   = params.Qnoise.s_dbgz;
    s_dbax   = params.Qnoise.s_dbax;
    s_dbay   = params.Qnoise.s_dbay;
    s_dbaz   = params.Qnoise.s_dbaz;
    s_dbmx   = params.Qnoise.s_dbmx;
    s_dbmy   = params.Qnoise.s_dbmy;
    s_dbmz   = params.Qnoise.s_dbmz;
    s_dbbaro = params.Qnoise.s_dbbaro;

    %-----------------------------
    % (4) 실제 계산 (심볼릭 자동생성 함수)
    %-----------------------------
    %  외력/추력/등은 일단 0
    Fa_x=0; Fa_y=0; Fa_z=0;
    Ft_x=0; Ft_y=0; Ft_z=0;
    Fr_x=0; Fr_y=0; Fr_z=0;
    Fn_x=0; Fn_y=0; Fn_z=0;

    Qeskf = calcQeskf_22_auto( ...
       px_hat, py_hat, pz_hat, ...
       u_hat, v_hat, w_hat, ...
       qw_hat, qx_hat, qy_hat, qz_hat, ...
       wx_hat, wy_hat, wz_hat, ...
       Fa_x, Fa_y, Fa_z, ...
       Ft_x, Ft_y, Ft_z, ...
       Fr_x, Fr_y, Fr_z, ...
       Fn_x, Fn_y, Fn_z, ...
       m, g, ...
       Ma_x, Ma_y, Ma_z, ...
       Mr_x, Mr_y, Mr_z, ...
       Mn_x, Mn_y, Mn_z, ...
       Cpx, ...
       Jx, Jy, Jz, ...
       tau_gyro, tau_acc, tau_mag, tau_baro, ...
       dt, ...
       s_px, s_py, s_pz, ...
       s_u, s_v, s_w, ...
       s_thx, s_thy, s_thz, ...
       s_wx_, s_wy_, s_wz_, ...
       s_dbgx, s_dbgy, s_dbgz, ...
       s_dbax, s_dbay, s_dbaz, ...
       s_dbmx, s_dbmy, s_dbmz, ...
       s_dbbaro ...
    );
end
