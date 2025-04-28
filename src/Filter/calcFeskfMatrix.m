function Feskf = calcFeskfMatrix(X_23, dt, params)
% calcFeskf22Matrix : ESKF(22차)용 F행렬(22x22)을 계산
%
% [입력]
%   - X_23(23x1) : [p(3), v(3), q(4), w(3), b_g(3), b_a(3), b_m(3), b_baro(1)]
%   - dt         : 샘플링 간격
%   - params     : 구조체 (질량, 관성, FOGM 타임상수, 외력/외란 등)
%
% [출력]
%   - F_22(22x22) : 선형화 행렬

    %-----------------------------
    % (1) Parse inputs
    %     공칭 + 바이어스 상태를 분해
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

    % (예: 바이어스 10개 — 필요에 따라 실제 보유 중인 bias 개수에 맞게 수정)
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
    % (2) 필요한 파라미터 가져오기
    %-----------------------------
    m  = params.vehicle.m_D;
    g  = params.environment.g(3);
    Jx = params.vehicle.J_X;
    Jy = params.vehicle.J_Y;
    Jz = params.vehicle.J_Z;

    tau_gyro = params.bias.tau_gyro;  % 예: 자이로 바이어스 FOGM 타임상수
    tau_acc  = params.bias.tau_acc;   % 예: 가속도
    tau_mag  = params.bias.tau_mag;   % 예: 마그네틱
    tau_baro = params.bias.tau_baro;  % 예: 기압계

    %-----------------------------
    % (3) 외력·모멘트 등 (필요시) 
    %     여기서는 일단 0으로 둠
    %-----------------------------
    Fa_x = 0; Fa_y = 0; Fa_z = 0;
    Ft_x = 0; Ft_y = 0; Ft_z = 0;
    Fr_x = 0; Fr_y = 0; Fr_z = 0;
    Fn_x = 0; Fn_y = 0; Fn_z = 0;
    Ma_x = 0; Ma_y = 0; Ma_z = 0;
    Mr_x = 0; Mr_y = 0; Mr_z = 0;
    Mn_x = 0; Mn_y = 0; Mn_z = 0;
    Cpx  = 0;

    %-----------------------------
    % (4) F행렬 계산 : 
    %     "calcFeskf_22_auto.m" (심볼릭 자동생성 결과) 호출
    %-----------------------------
    Feskf = calcFeskf_22_auto( ...
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
        tau_gyro, tau_acc, tau_mag, tau_baro, ...  % 바이어스 타임상수
        dt ...
    );
end
