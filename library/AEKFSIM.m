function [ X, P, rawdata, xhat ] = AEKFSIM( z, P, X, Ut, params, dt )

    persistent Qk0 Rk0

    if isempty(Qk0)

        qs_x = 0;
        qs_y = 0;
        qs_u = cmd(4) + 0.1;
        qs_v = cmd(5) + 0.1;
        qs_psi = deg2rad(0);
        qs_r = deg2rad(cmd(6)) + deg2rad(0.1);
    
        Qs = diag( [ qs_x.^2, qs_y.^2, qs_u.^2, qs_v.^2, qs_psi.^2, qs_r.^2 ] );
        
        q_x = Qs(1,1);
        q_y = Qs(2,2);
        % q_u = cmd(4) + Qs(3,3);
        % q_v = cmd(5) + Qs(4,4);
        q_u = Qs(3,3);
        q_v = Qs(4,4);
        q_psi = Qs(5,5);
        q_r = Qs(6,6);
        % q_r = deg2rad(cmd(6)) + Qs(6,6);
    
        Qk = zeros(6,6);
    
        % Define the diagonal elements
        Qk(1,1) = (q_u * cos(X(5))^2 / 3 + q_v * sin(X(5))^2 / 3 + q_psi * (X(4) * cos(X(5)) + X(3) * sin(X(5)))^2 / 3) * dt^3 + q_x * dt;
        Qk(2,2) = (q_v * cos(X(5))^2 / 3 + q_u * sin(X(5))^2 / 3 + q_psi * (X(3) * cos(X(5)) - X(4) * sin(X(5)))^2 / 3) * dt^3 + q_y * dt;
        Qk(3,3) = (q_v * X(6)^2 / 3 + q_r * X(4)^2 / 3) * dt^3 + q_u * dt;
        Qk(4,4) = (q_u * X(6)^2 / 3 + q_r * X(3)^2 / 3) * dt^3 + q_v * dt;
        Qk(5,5) = q_r * dt^3 / 3 + q_psi * dt;
        Qk(6,6) = dt * q_r;
    
        % Define the off-diagonal elements
        Qk(1,2) = -dt^3 * (q_psi * (X(4) * cos(X(5)) + X(3) * sin(X(5))) * (X(3) * cos(X(5)) - X(4) * sin(X(5))) / 3 - q_u * cos(X(5)) * sin(X(5)) / 3 + q_v * cos(X(5)) * sin(X(5)) / 3);
        Qk(1,3) = X(6) * q_v * sin(X(5)) * dt^3 / 3 + q_u * cos(X(5)) * dt^2 / 2;
        Qk(1,4) = X(6) * q_u * cos(X(5)) * dt^3 / 3 - q_v * sin(X(5)) * dt^2 / 2;
        Qk(1,5) = -dt^2 * q_psi * (X(4) * cos(X(5)) + X(3) * sin(X(5))) / 2;
        Qk(1,6) = 0;
        Qk(2,3) = -X(6) * q_v * cos(X(5)) * dt^3 / 3 + q_u * sin(X(5)) * dt^2 / 2;
        Qk(2,4) = X(6) * q_u * sin(X(5)) * dt^3 / 3 + q_v * cos(X(5)) * dt^2 / 2;
        Qk(2,5) = dt^2 * q_psi * (X(3) * cos(X(5)) - X(4) * sin(X(5))) / 2;
        Qk(2,6) = 0;
        Qk(3,4) = -q_r * X(3) * X(4) * dt^3 / 3 + (X(6) * q_u / 2 - X(6) * q_v / 2) * dt^2;
        Qk(3,5) = -dt^3 * q_r * X(4) / 3;
        Qk(3,6) = -dt^2 * q_r * X(4) / 2;
        Qk(4,5) = dt^3 * q_r * X(3) / 3;
        Qk(4,6) = dt^2 * q_r * X(3) / 2;
        Qk(5,6) = dt^2 * q_r / 2;

        % Make the matrix symmetric
        Qk = Qk + triu(Qk,1)';

        Qk0 = Qk;

    end

    if isempty(Rk0)

        Rq_x = 2.365;
        Rq_y = 2.365;
        Rq_u = 9.2;
        Rq_v = 9.2;
        Rq_psi = deg2rad(1);
        Rq_r = deg2rad(1);
    
        Rk0 = diag( [ Rq_x.^2, Rq_y.^2, Rq_u.^2, Rq_v.^2, Rq_psi.^2, Rq_r.^2 ] );

        FL2 = 1;

    end


    Iz = params.Airbearing.I_Z;
    D = params.Airbearing.D_ref;
    M = params.Airbearing.m;
    T = params.Airbearing.Thrust;

    idle = 49;

    Ut(9) = cmd2tq( Ut(9), idle, params, dt );

    rawdata = z;
    
             
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

     
    xhat = F * X + B * Ut;
    P = F * P * F' + Qk';

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
    X = xhat + K * yhat;
    P = ( eye(6) - K * H ) * P * ( eye( 6 ) - K * H )' + K * R * K'; % Joseph form

    Qk = 

end