function [X, P, yhat, xhat] = processDataSIMUKF(z, P, X, Ut, params, dt)
    % UKF parameters
    n = length(X); % State dimension
    m = length(z); % Measurement dimension
    alpha = 1e-3; % Spread of sigma points
    ki = 0; % Secondary scaling parameter
    beta = 2; % Incorporates prior knowledge of the distribution
    lambda = alpha^2 * (n + ki) - n;
    
    % Weights for means and covariances
    Wm = [lambda / (n + lambda), repmat(1 / (2*(n + lambda)), 1, 2*n)];
    Wc = Wm;
    Wc(1) = Wc(1) + (1 - alpha^2 + beta);
    
    % Extract parameters
    Iz = params.Airbearing.I_Z;
    D = params.Airbearing.D_ref;
    M = params.Airbearing.m;
    T = params.Airbearing.Thrust;
    idle = 49;
    
    % Process noise
    q_pos = 0; % Position noise
    q_vel = 11; % Velocity noise
    q_att = deg2rad(0); % Attitude noise
    q_r = deg2rad(40); % Angular rate noise
    
    % Generate sigma points
    sP = chol((n + lambda) * P)';
    Xsigma = [X, X + sP, X - sP];
    
    % Propagate sigma points
    for i = 1:2*n+1
        r = Xsigma(1:2, i);
        V_B = Xsigma(3:4, i);
        att = Xsigma(5, i);
        R = Xsigma(6, i);
        
        % Control Vector decomposition
        Open_Valve = Ut(1:8);
        RW_tau = cmd2tq(Ut(9), idle, params, dt);
        Control_Logic = T .* [0, -1, -1, 0, 0, 1, 1, 0;
                              -1, 0, 0, 1, 1, 0, 0, -1;
                              -D, D, -D, D, -D, D, -D, D];
        final_FT = Control_Logic * Open_Valve;
        Fx = final_FT(1);
        Fy = final_FT(2);
        Tau = final_FT(3) + RW_tau;
        
        % State derivatives
        r_dot = [V_B(1) * cos(att) - V_B(2) * sin(att);
                 V_B(1) * sin(att) + V_B(2) * cos(att)];
        V_B_dot = [Fx / M - R * V_B(2);
                   Fy / M + R * V_B(1)];
        att_dot = R;
        R_dot = Tau / Iz;
        
        % Update sigma points
        Xsigma(:, i) = Xsigma(:, i) + [r_dot; V_B_dot; att_dot; R_dot] * dt;
    end
    
    % Predict new state and covariance
    xhat = zeros(n, 1);
    P = zeros(n, n);
    for i = 1:2*n+1
        xhat = xhat + Wm(i) * Xsigma(:,i);
    end
    for i = 1:2*n+1
        P = P + Wc(i) * (Xsigma(:,i) - xhat) * (Xsigma(:,i) - xhat)';
    end
    
    % Add process noise
    Q = diag([q_pos^2, q_pos^2, q_vel^2, q_vel^2, q_att^2, q_r^2]) * dt;
    P = P + Q;
    
    % Measurement update
    Zsigma = Xsigma([1,2,3,4,5,6],:); % Full state measurement model
    zhat = zeros(m, 1);
    for i = 1:2*n+1
        zhat = zhat + Wm(i) * Zsigma(:,i);
    end
    
    Pzz = zeros(m, m);
    Pxz = zeros(n, m);
    for i = 1:2*n+1
        Pzz = Pzz + Wc(i) * (Zsigma(:,i) - zhat) * (Zsigma(:,i) - zhat)';
        Pxz = Pxz + Wc(i) * (Xsigma(:,i) - xhat) * (Zsigma(:,i) - zhat)';
    end
    
    % Measurement noise covariance
    R = diag([0.365^2, 0.365^2, 1^2, 1^2, deg2rad(0.025)^2, deg2rad(1)^2]);
    Pzz = Pzz + R;
    
    % Kalman gain
    K = Pxz / Pzz;
    
    % Update state and covariance
    yhat = z - zhat;
    X = xhat + K * yhat;
    P = P - K * Pzz * K';
end