function [ Xk, Pk, rawdata, xhat ] = processDataSIM_tunning( z, X, P, Ut, params, dt )

    Iz = params.Airbearing.I_Z;
    D = params.Airbearing.D_ref;
    M = params.Airbearing.m;
    T = params.Airbearing.Thrust;
    rawdata = z;

    idle = 49;

    Ut(9) = cmd2tq( Ut(9), idle, params, dt );

    Rq_x = 0.365;
    Rq_y = 0.365;
    Rq_u = 7.2;
    Rq_v = 7.2;
    Rq_psi = deg2rad(0.025);
    Rq_r = deg2rad(1);


    R = diag( [ Rq_x.^2, Rq_y.^2, Rq_u.^2, Rq_v.^2, Rq_psi.^2, Rq_r.^2 ] );
             
    % Given values
    
     F =  [
            1, 0, dt*cos(X(5)), -dt*sin(X(5)),   dt * ( -X(3)*sin(X(5)) - X(4)*cos(X(5)) ),           0;
            0, 1, dt*sin(X(5)),  dt*cos(X(5)),   dt * (  X(3)*cos(X(5)) - X(4)*sin(X(5)) ),           0;
            0, 0,            1,      -dt*X(6),                                           0,  -dt * X(4);
            0, 0,      dt*X(6),             1,                                           0,   dt * X(3);
            0, 0,            0,             0,                                           1,          dt;
            0, 0,            0,             0,                                           0,           1
         ];


    B = [
            0,          0,           0,          0,           0,          0,           0,          0       0;
            0,          0,           0,          0,           0,          0,           0,          0       0;
            0,       -T/M,        -T/M,          0,           0,        T/M,         T/M,          0       0;
         -T/M,          0,           0,        T/M,         T/M,          0,           0,       -T/M       0;
            0,          0,           0,          0,           0,          0,           0,          0       0;
    -(D*T)/Iz,   (D*T)/Iz,   -(D*T)/Iz,   (D*T)/Iz,   -(D*T)/Iz,   (D*T)/Iz,   -(D*T)/Iz,   (D*T)/Iz     1/Iz
         ] .* dt;


    cmd = B*Ut;

    qs_x = 0;
    qs_y = 0;
    qs_u = 40;
    qs_v = 40;
    qs_psi = deg2rad(0);
    qs_r = deg2rad(15);

    Qs = diag( [ qs_x.^2, qs_y.^2, qs_u.^2, qs_v.^2, qs_psi.^2, qs_r.^2 ] );
    
    q_x = Qs(1,1);
    q_y = Qs(2,2);
    q_u = Qs(3,3);
    q_v = Qs(4,4);
    q_psi = Qs(5,5);
    q_r = Qs(6,6);

     % Define Qk matrix
    Qk = zeros(6,6);
    
    % Define all elements
    Qk(1,1) = dt^3*((q_v*sin(X(5))^2)/3 - (q_u*(sin(X(5))^2 - 1))/3);
    Qk(1,2) = (dt^3*sin(2*X(5))*(q_u - q_v))/6;
    Qk(1,3) = (X(6)*q_v*sin(X(5))*dt^3)/3 + (q_u*cos(X(5))*dt^2)/2;
    Qk(1,4) = (X(6)*q_u*cos(X(5))*dt^3)/3 - (q_v*sin(X(5))*dt^2)/2;
    Qk(1,5) = 0;
    Qk(1,6) = 0;
    
    Qk(2,1) = Qk(1,2);
    Qk(2,2) = dt^3*((q_u*sin(X(5))^2)/3 - (q_v*(sin(X(5))^2 - 1))/3);
    Qk(2,3) = - (X(6)*q_v*cos(X(5))*dt^3)/3 + (q_u*sin(X(5))*dt^2)/2;
    Qk(2,4) = (X(6)*q_u*sin(X(5))*dt^3)/3 + (q_v*cos(X(5))*dt^2)/2;
    Qk(2,5) = 0;
    Qk(2,6) = 0;
    
    Qk(3,1) = Qk(1,3);
    Qk(3,2) = Qk(2,3);
    Qk(3,3) = ((q_v*X(6)^2)/3 + (q_r*X(4)^2)/3)*dt^3 + q_u*dt;
    Qk(3,4) = - (q_r*X(3)*X(4)*dt^3)/3 + ((X(6)*q_u)/2 - (X(6)*q_v)/2)*dt^2;
    Qk(3,5) = -(dt^3*q_r*X(4))/3;
    Qk(3,6) = -(dt^2*q_r*X(4))/2;
    
    Qk(4,1) = Qk(1,4);
    Qk(4,2) = Qk(2,4);
    Qk(4,3) = Qk(3,4);
    Qk(4,4) = ((q_u*X(6)^2)/3 + (q_r*X(3)^2)/3)*dt^3 + q_v*dt;
    Qk(4,5) = (dt^3*q_r*X(3))/3;
    Qk(4,6) = (dt^2*q_r*X(3))/2;
    
    Qk(5,1) = Qk(1,5);
    Qk(5,2) = Qk(2,5);
    Qk(5,3) = Qk(3,5);
    Qk(5,4) = Qk(4,5);
    Qk(5,5) = (dt^3*q_r)/3;
    Qk(5,6) = (dt^2*q_r)/2;
    
    Qk(6,1) = Qk(1,6);
    Qk(6,2) = Qk(2,6);
    Qk(6,3) = Qk(3,6);
    Qk(6,4) = Qk(4,6);
    Qk(6,5) = Qk(5,6);
    Qk(6,6) = dt*q_r;

                        
    xhat = F * X + B * Ut;
    P = F * P * F' + Qk;

    % Measurement model
    H = [ 1, 0, 0, 0, 0, 0 ;
          0, 1, 0, 0, 0, 0 ;
          0, 0, 0, 0, 0, 0 ;
          0, 0, 0, 0, 0, 0 ;
          0, 0, 0, 0, 1, 0 ;
          0, 0, 0, 0, 0, 0 ];

    h = H * xhat;

    % Compute innovation
    yhat = z - h;

    % Kalman gain
    S = H * P * H' + R;
    K = P * H' * inv(S);
 
    % Update
    Xk = xhat + K * yhat;
    Pk = ( eye(6) - K * H ) * P * ( eye( 6 ) - K * H )' + K * R * K'; % Joseph form

end