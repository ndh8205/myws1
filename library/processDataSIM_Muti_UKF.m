function [ X, P, yhat, xhat ] = processDataSIM_Muti_UKF( z, P, X, X_target, Ut, params, dt )
   
    n = length(X);
    m = length(z);

    qs_u = 0.095;
    qs_v = 0.095;
    qs_r = deg2rad(0.095);

    % Unscented Kalman Filter parameter set
    alpha = 1e-3;
    beta = 2;
    kappa = 0;

    lambda = alpha^2 * (n + kappa) - n;

    % weight
    Wm = zeros(2*n + 1,1); % mean
    Wc = zeros(2*n + 1,1); % covariance

    Wm(1) = lambda / (n + lambda);
    Wc(1) = Wm(1) + (1 - alpha^2 + beta);

    for i = 2:2*n+1
        Wm(i) = 1 / (2*(n + lambda));
        Wc(i) = Wm(i);
    end


    % sigma point generate
    sigma_pts = zeros(n, 2*n + 1);
    sigma_pts(:,1) = X;
    sqrtP = chol((n + lambda)*P, 'lower');

    for i = 1:n
        sigma_pts(:, i+1) = X + sqrtP(:,i);
        sigma_pts(:, n + i + 1) = X - sqrtP(:,i);
    end

    % Qk def

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

    % measurement noise matrix
    Rq_rho = 5;
    Rq_theta = deg2rad(5);
    Rq_rm = deg2rad(0.1);
    Rs = diag( [ Rq_rho.^2, Rq_theta.^2, Rq_rm.^2 ] );
    R = Rs;


    % sigma point predict
    sigma_pts_pred = zeros(n, 2*n + 1);

    for i = 1:2*n+1
        x = sigma_pts(:,i);
        xdot = vehicle_dynamics_Airbearing_stochastic( x, Ut, zeros(3,1), dt, params );
        x_pred = x + xdot * dt;
        sigma_pts_pred(:,i) = x_pred;
    end

    % predict state
    xhat = zeros(n,1);
    for i = 1:2*n+1
        xhat = xhat + Wm(i) * sigma_pts_pred(:,i);
    end

    % predict covariance
    P_hat = zeros(n,n);
    for i = 1:2*n+1
        dx = sigma_pts_pred(:,i) - xhat;
        P_hat = P_hat + Wc(i) * (dx) * (dx)';
    end

    % P hat
    P_hat = P_hat + Qk;

    % measurement predict
    Z_sigma = zeros(m, 2*n + 1);
    for i = 1:2*n+1
        x = sigma_pts_pred(:,i);
        % range-bearing measurement_model update
        Z_sigma(:,i) = measurement_model_RB(x, X_target);
    end

    % predict measurement
    zhat = zeros(m,1);
    for i = 1:2*n+1
        zhat = zhat + Wm(i) * Z_sigma(:,i);
    end

    % measurment covariance

    P_zz = zeros(m,m);
    for i = 1:2*n+1
        dz = Z_sigma(:,i) - zhat;
        P_zz = P_zz + Wc(i) * dz * dz';
    end

    P_zz = P_zz + R;

    % state & measurement covariance
    P_xz = zeros(n,m);
    for i = 1:2*n+1
        dx = sigma_pts_pred(:,i) - xhat;
        dz = Z_sigma(:,i) - zhat;
        P_xz = P_xz + Wc(i) * dx * dz';
    end

    % kalman gain
    K = P_xz / P_zz;

    % state update
    yhat = z - zhat;
    yhat(2) = wrapToPi(yhat(2));
    yhat(3) = wrapToPi(yhat(3));

    X = xhat + K * yhat;
    X(5) = wrapToPi(X(5));
    
    % covariance update
    P = P_hat - K * P_zz * K';

end
