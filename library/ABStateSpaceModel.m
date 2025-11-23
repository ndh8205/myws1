% State Space 모델 정의 함수
function [A, B] = ABStateSpaceModel(X_ss, params)

    u = X_ss(3);
    v = X_ss(4);
    psi = X_ss(5);
    r = X_ss(6);
    
    m = params.Airbearing.m;
    Iz = params.Airbearing.I_Z;
    d = params.Airbearing.D_ref;
    T = params.Airbearing.Thrust;
    
    A = [0 0 cos(psi) -sin(psi) -u*sin(psi)-v*cos(psi) 0;
         0 0 sin(psi)  cos(psi)  u*cos(psi)-v*sin(psi) 0;
         0 0    0        -r            0             -v;
         0 0    r         0            0              u;
         0 0    0         0            0              1;
         0 0    0         0            0              0];

    B = [0      0      0      0      0      0      0      0;
         0      0      0      0      0      0      0      0;
         0   -T/m   -T/m      0      0    T/m    T/m     0;
        -T/m    0      0    T/m    T/m      0      0   -T/m;         
         0      0      0      0      0      0      0      0;
        -T*d/Iz T*d/Iz -T*d/Iz T*d/Iz -T*d/Iz T*d/Iz -T*d/Iz T*d/Iz];
end