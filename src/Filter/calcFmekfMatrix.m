function Fmekf = calcFmekfMatrix(X, dt, params)
% calcFmekfMatrix : MEKF용 F행렬(12x12)을 계산
% 
% [입력]
%   - X(13x1) : [px, py, pz, u, v, w, qw, qx, qy, qz, wx, wy, wz]
%   - dt      : 샘플링 간격
%   - params  : 구조체 (질량, 관성, etc.)
%
% [출력]
%   - Fmekf(12x12)

    % -----------------------------
    % 1) Parse inputs
    px = X(1);   py = X(2);   pz = X(3);
    u  = X(4);   v  = X(5);   w  = X(6);
    qw = X(7);   qx = X(8);   qy = X(9);   qz = X(10);
    wx = X(11);  wy = X(12);  wz = X(13);

    m  = params.vehicle.m_D;
    g  = params.environment.g(3);
    Jx = params.vehicle.J_X;
    Jy = params.vehicle.J_Y;
    Jz = params.vehicle.J_Z;

    % -----------------------------
    % 2) 오차 상태 F행렬 계산
    %   - 이미 심볼릭 전개로 만들어진 "calcFmekf_auto" 호출
    %   - 공력/추력/외력 등을 0이나 적절한 값으로 대입 가능
    %   - 여기서는 일단 모두 0으로 둠

    Fa_x = 0;  Fa_y = 0;  Fa_z = 0;
    Ft_x = 0;  Ft_y = 0;  Ft_z = 0;
    Fr_x = 0;  Fr_y = 0;  Fr_z = 0;
    Fn_x = 0;  Fn_y = 0;  Fn_z = 0;
    Ma_x = 0;  Ma_y = 0;  Ma_z = 0;
    Mr_x = 0;  Mr_y = 0;  Mr_z = 0;
    Mn_x = 0;  Mn_y = 0;  Mn_z = 0;
    Cpx  = 0;

    % (만약 params 구조체에 실제 힘/모멘트 값이 있으면 여기서 대입)

    % -----------------------------
    % 3) 실제 계산 (심볼릭 자동생성 함수)
    Fmekf = calcFmekf_auto( ...
        px, py, pz, ...
        u, v, w, ...
        qw, qx, qy, qz, ...
        wx, wy, wz, ...
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
        dt ...
    );

end
