function [ X, P, yhat, xhat, S ] = processDataSIM_Muti_EKF_CA_imm( z, P, X, X_target, Ut, params, dt )
    % Process noise intensity (jerk variance)
    sigma_j = 1185; % Adjust this value as needed
    q = sigma_j^2;

    % State transition matrix F for CA model
    F = [
        1, 0, dt,  0, 0.5*dt^2,         0;
        0, 1,  0, dt,        0, 0.5*dt^2;
        0, 0,  1,  0,       dt,         0;
        0, 0,  0,  1,        0,        dt;
        0, 0,  0,  0,        1,         0;
        0, 0,  0,  0,        0,         1;
    ];

    % Process noise covariance Qk for CA model
    Qk = zeros(6,6);
    Qk(1,1) = q * dt^5 / 20;
    Qk(1,3) = q * dt^4 / 8;
    Qk(1,5) = q * dt^3 / 6;
    Qk(3,1) = Qk(1,3);
    Qk(3,3) = q * dt^3 / 3;
    Qk(3,5) = q * dt^2 / 2;
    Qk(5,1) = Qk(1,5);
    Qk(5,3) = Qk(3,5);
    Qk(5,5) = q * dt;

    Qk(2,2) = q * dt^5 / 20;
    Qk(2,4) = q * dt^4 / 8;
    Qk(2,6) = q * dt^3 / 6;
    Qk(4,2) = Qk(2,4);
    Qk(4,4) = q * dt^3 / 3;
    Qk(4,6) = q * dt^2 / 2;
    Qk(6,2) = Qk(2,6);
    Qk(6,4) = Qk(4,6);
    Qk(6,6) = q * dt;

    % Prediction step
    xhat = F * X;
    P = F * P * F' + Qk;

    % Measurement model remains the same
    dxm = xhat(1) - X_target(1);
    dym = xhat(2) - X_target(2);
    range_miner = sqrt( dxm^2 + dym^2 );

    % H - Range-bearing observation model
    H = zeros(3, 6);
    H(1,1) = dxm / range_miner;
    H(1,2) = dym / range_miner;
    H(2,1) = -dym / ( dxm^2 + dym^2 );
    H(2,2) = dxm / ( dxm^2 + dym^2 );
    H(2,5) = -1;
    H(3,6) = 1;

    % Measurement prediction
    h = measurement_model_RB( xhat, X_target );

    % Innovation
    yhat = z - h;
    yhat(2) = wrapToPi(yhat(2));
    yhat(3) = wrapToPi(yhat(3));

    % Measurement noise covariance R (using your original values)
    Rq_rho = 5;
    Rq_theta = deg2rad(5);
    Rq_rm = deg2rad(0.1);

    Rs = diag( [ Rq_rho.^2, Rq_theta.^2, Rq_rm.^2 ] );
    R = Rs;

    % Kalman gain
    S = H * P * H' + R;
    K = P * H' / S;

    % Update
    X = xhat + K * yhat;
    X(5) = wrapToPi( X(5) );
    X(6) = wrapToPi( X(6) );

    P = ( eye(6) - K * H ) * P * ( eye( 6 ) - K * H )' + K * R * K'; % Joseph form
end
