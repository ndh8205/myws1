function [ xdot, F_total ] = vehicle_dynamics_Airbearing_AXBU( X, Ut, params, dt )

    Iz = params.Airbearing.I_Z;
    D = params.Airbearing.D_ref;
    M = params.Airbearing.m;
    T = params.Airbearing.Thrust;


    % Given values
    
     % F =  [
     %        1, 0, dt*cos(X(5)), -dt*sin(X(5)),   dt * ( -X(3)*sin(X(5)) - X(4)*cos(X(5)) ),           0;
     %        0, 1, dt*sin(X(5)),  dt*cos(X(5)),   dt * (  X(3)*cos(X(5)) - X(4)*sin(X(5)) ),           0;
     %        0, 0,            1,      -dt*X(6),                                           0,  -dt * X(4);
     %        0, 0,      dt*X(6),             1,                                           0,   dt * X(3);
     %        0, 0,            0,             0,                                           1,          dt;
     %        0, 0,            0,             0,                                           0,           1
     %     ];
    % 
    A = [
            0,  0,    cos(X(5)),   -sin(X(5)),  -X(3)*sin(X(5))-X(4)*cos(X(5)),      0;
            0,  0,    sin(X(5)),    cos(X(5)),   X(3)*cos(X(5))-X(4)*sin(X(5)),      0;
            0,  0,            0,        -X(6),                               0,  -X(4);
            0,  0,         X(6),            0,                               0,   X(3);
            0,  0,            0,            0,                               0,      1;
            0,  0,            0,            0,                               0,      0 ];


    % A = [
    %     0,  0,    cos(X(5)),   -sin(X(5)),   0,      0;
    %     0,  0,    sin(X(5)),    cos(X(5)),   0,      0;
    %     0,  0,        -X(6),            0,   0,      0;
    %     0,  0,            0,         X(6),   0,      0;
    %     0,  0,            0,            0,   0,      1;
    %     0,  0,            0,            0,   0,      0 ];

    % A = [
    %     0,  0,    cos(X(5)),   -sin(X(5)),   0,      0;
    %     0,  0,    sin(X(5)),    cos(X(5)),   0,      0;
    %     0,  0,            0,        -X(6),   0,      0;
    %     0,  0,         X(6),            0,   0,      0;
    %     0,  0,            0,            0,   0,      1;
    %     0,  0,            0,            0,   0,      0 ];

    % % A의 거듭제곱 계산
    % A2 = A^2;
    % A3 = A^3;
    % A4 = A^4;
    % 
    % % 4차까지의 테일러 급수 전개
    % Ad = eye(6) + A*dt + (A2*dt^2)/2 + (A3*dt^3)/6 + (A4*dt^4)/24;


    % B = [
    %         0,          0,           0,          0,           0,          0,           0,          0       0;
    %         0,          0,           0,          0,           0,          0,           0,          0       0;
    %         0,       -T/M,        -T/M,          0,           0,        T/M,         T/M,          0       0;
    %      -T/M,          0,           0,        T/M,         T/M,          0,           0,       -T/M       0;
    %         0,          0,           0,          0,           0,          0,           0,          0       0;
    % -(D*T)/Iz,   (D*T)/Iz,   -(D*T)/Iz,   (D*T)/Iz,   -(D*T)/Iz,   (D*T)/Iz,   -(D*T)/Iz,   (D*T)/Iz     1/Iz
    %      ] .* dt;

    B = [
            0,          0,           0,          0,           0,          0,           0,          0       0;
            0,          0,           0,          0,           0,          0,           0,          0       0;
            0,       -T/M,        -T/M,          0,           0,        T/M,         T/M,          0       0;
         -T/M,          0,           0,        T/M,         T/M,          0,           0,       -T/M       0;
            0,          0,           0,          0,           0,          0,           0,          0       0;
    -(D*T)/Iz,   (D*T)/Iz,   -(D*T)/Iz,   (D*T)/Iz,   -(D*T)/Iz,   (D*T)/Iz,   -(D*T)/Iz,   (D*T)/Iz     1/Iz
         ];


    F_total = B*Ut;


    xdot = A * X + B * Ut;

end