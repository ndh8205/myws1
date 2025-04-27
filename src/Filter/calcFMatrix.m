function F = calcFMatrix(X, dt, params)
    %----------------------------------------------------------
    % calcFMatrix:
    %  Use the final symbolic partial derivatives (A_sym) 
    %  that you already computed (and displayed).
    %  Then discretize:  F = I + A*dt.
    %
    % State X = [ px, py, pz,  u, v, w,  qw, qx, qy, qz,  wx, wy, wz ]^T
    %
    % The result matches the large 13x13 matrix you showed:
    %  A_sym = d f_cont / dX
    %----------------------------------------------------------

    g_ = params.environment.g(3);
    Jx_ = params.vehicle.J_X;
    Jy_ = params.vehicle.J_Y;
    Jz_ = params.vehicle.J_Z;
    
    % 1) Parse state
    px = X(1);  py = X(2);  pz = X(3);
    u  = X(4);  v  = X(5);  w  = X(6);
    qw = X(7);  qx = X(8);  qy = X(9);  qz = X(10);
    wx = X(11); wy = X(12); wz = X(13);

    % 2) Build A (13x13) EXACTLY as in your final symbolic result
    A = zeros(13);

    % --- row1
    A(1,4)  = (qw^2 + qx^2 - qy^2 - qz^2);
    A(1,5)  = 2*(qx*qy - qw*qz);
    A(1,6)  = 2*(qw*qy + qx*qz);
    A(1,7)  = 2*qw*u - 2*qz*v + 2*qy*w;
    A(1,8)  = 2*qx*u + 2*qy*v + 2*qz*w;
    A(1,9)  = 2*qx*v - 2*qy*u + 2*qw*w;
    A(1,10) = 2*qx*w - 2*qw*v - 2*qz*u;

    % --- row2
    A(2,4)  = 2*(qw*qz + qx*qy);
    A(2,5)  = (qw^2 - qx^2 + qy^2 - qz^2);
    A(2,6)  = 2*(qy*qz - qw*qx);
    A(2,7)  = 2*qz*u + 2*qw*v - 2*qx*w;
    A(2,8)  = 2*qy*u - 2*qx*v - 2*qw*w;
    A(2,9)  = 2*qx*u + 2*qy*v + 2*qz*w;
    A(2,10) = 2*qw*u - 2*qz*v + 2*qy*w;

    % --- row3
    A(3,4)  = 2*(qx*qz - qw*qy);
    A(3,5)  = 2*(qw*qx + qy*qz);
    A(3,6)  = (qw^2 - qx^2 - qy^2 + qz^2);
    A(3,7)  = 2*qx*v - 2*qy*u + 2*qw*w;
    A(3,8)  = 2*qz*u + 2*qw*v - 2*qx*w;
    A(3,9)  = 2*qz*v - 2*qw*u - 2*qy*w;
    A(3,10) = 2*qx*u + 2*qy*v + 2*qz*w;

    % --- row4  (dot u)
    %   = [0,0,0, 0, wz, -wy, -2*g*qy,  2*g*qz, -2*g*qw, 2*g*qx, 0, -w, v]
    A(4,5)   = wz;
    A(4,6)   = -wy;
    A(4,7)   = -2*g_*qy; 
    A(4,8)   =  2*g_*qz;
    A(4,9)   = -2*g_*qw;
    A(4,10)  =  2*g_*qx;
    A(4,12)  = -w;
    A(4,13)  =  v;

    % --- row5  (dot v)
    %   = [-wz, 0, wx, 2*g*qx, 2*g*qw, 2*g*qz, 2*g*qy, w, 0, -u ]
    A(5,4)   = -wz;
    A(5,6)   =  wx;
    A(5,7)   =  2*g_*qx;
    A(5,8)   =  2*g_*qw;
    A(5,9)   =  2*g_*qz;
    A(5,10)  =  2*g_*qy;
    A(5,11)  =  w;
    A(5,13)  = -u;

    % --- row6  (dot w)
    %   = [ wy, -wx, 0, 2*g*qw, -2*g*qx, -2*g*qy, 2*g*qz, -v, u, 0]
    A(6,4)   =  wy;
    A(6,5)   = -wx;
    A(6,7)   =  2*g_*qw;
    A(6,8)   = -2*g_*qx;
    A(6,9)   = -2*g_*qy;
    A(6,10)  =  2*g_*qz;
    A(6,12)  =  u;
    A(6,11)  = -v;

    % --- row7 (dot qw)
    %   = [ 0,0,0, 0,0,0, 0, -wx/2, -wy/2, -wz/2, -qx/2, -qy/2, -qz/2]
    A(7,8)   = -wx/2;
    A(7,9)   = -wy/2;
    A(7,10)  = -wz/2;
    A(7,11)  = -qx/2;
    A(7,12)  = -qy/2;
    A(7,13)  = -qz/2;

    % row8 (dot qx)
    A(8,7)   =  wx/2;
    A(8,9)   =  wz/2;
    A(8,10)  = -wy/2;
    A(8,11)  =  qw/2;
    A(8,12)  = -qz/2;
    A(8,13)  =  qy/2;

    % row9 (dot qy)
    A(9,7)   =  wy/2;
    A(9,8)   = -wz/2;
    A(9,10)  =  wx/2;
    A(9,11)  =  qz/2;
    A(9,12)  =  qw/2;
    A(9,13)  = -qx/2;

    % row10 (dot qz)
    A(10,7)  =  wz/2;
    A(10,8)  =  wy/2;
    A(10,9)  = -wx/2;
    A(10,11) = -qy/2;
    A(10,12) =  qx/2;
    A(10,13) =  qw/2;

    % --- row11~13 => dot(omega)
    % (row11) dot wx
    A(11,13) = ( (Jy_ - Jz_)/Jx_ )*wz;  
    A(11,12) = ( (Jy_ - Jz_)/Jx_ )*wy;

    % (row12) dot wy
    A(12,11) = -( (Jx_ - Jz_)/Jy_ )*wz;
    A(12,13) = -( (Jx_ - Jz_)/Jy_ )*wx;

    % (row13) dot wz
    A(13,12) = ( (Jx_ - Jy_)/Jz_ )*wy;
    A(13,11) = ( (Jx_ - Jy_)/Jz_ )*wx;

    % 3) Discretize => F = I + A * dt (first order)
    I13 = eye(13);
    F = I13 + A * dt;
end