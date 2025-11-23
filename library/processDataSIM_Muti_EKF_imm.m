function [ X, P, yhat, xhat, S ] = processDataSIM_Muti_EKF_imm( z, P, X, X_target, Ut, params, dt )

    Iz = params.Airbearing.I_Z;
    D = params.Airbearing.D_ref;
    M = params.Airbearing.m;
    T = params.Airbearing.Thrust;

    idle = 49;

    Ut(9) = cmd2tq_origin( Ut(9), idle, params, dt );

    Rq_rho = 5;
    Rq_theta = deg2rad(1);
    Rq_rm = deg2rad(5);

    qs_u = 0.51/2;
    qs_v = 0.51/2;
    qs_r = deg2rad(0.65/2);

    Rs = diag( [ Rq_rho.^2, Rq_theta.^2, Rq_rm.^2 ] );

    

    R = Rs;

    Amat_ND =  [
        1, 0, dt*cos(X(5)), -dt*sin(X(5)),    0,           0;
        0, 1, dt*sin(X(5)),  dt*cos(X(5)),    0,           0;
        0, 0,                 1,      -dt*X(6),    0,           0;
        0, 0,      dt*X(6),                  1,    0,           0;
        0, 0,                 0,                  0,    1,      dt;
        0, 0,                 0,                  0,    0,           1
     ];


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

    qs_x = 0;
    qs_y = 0;
    qs_psi = deg2rad(0);

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
   

    % The matrix is now fully defined as per the given structure and original notation
    xhat = Amat_ND * X + B * Ut;
    % xhat = srk4( @vehicle_dynamics_Airbearing_stochastic, X, Ut, zeros(3,1), dt, params, dt );
    P = Amat_ND * P * Amat_ND' + Qk;

    dxm = xhat(1) - X_target(1);
    dym = xhat(2) - X_target(2);

    % Inertial frame range
    range_miner = sqrt( dxm^2 + dym^2 );

    % H - Range-bearing observation model (2d)
    H = zeros(3, 6);
    
    % First row
    H(1,1) = dxm / range_miner;
    H(1,2) = dym / range_miner;
    
    % Second row
    H(2,1) = -dym / ( dxm^2 + dym^2 );
    H(2,2) = dxm / ( dxm^2 + dym^2 );
    H(2,5) = -1;

    % Third row
    H(3,6) = 1;

    % Construct observation vector
    h = measurement_model_RB( xhat, X_target );

    % Compute innovation
    yhat = z - h;
    yhat(2) = wrapToPi(yhat(2));
    yhat(3) = wrapToPi(yhat(3));

    % Kalman gain
    S = H * P * H' + R;
    K = P * H' * inv(S);
 
    % Update
    X = xhat + K * yhat;
    X(5) = wrapToPi( X(5) );
    X(6) = wrapToPi( X(6) );

    P = ( eye(6) - K * H ) * P * ( eye( 6 ) - K * H )' + K * R * K'; % Joseph form


end