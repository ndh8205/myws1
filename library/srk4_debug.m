function [x, F_total] = srk4_debug(Func, x, u, Q, dt, params, delt)
    % Scaling factor for stochastic integration
    alpha = [1.0/6.0; 2.0/6.0; 2.0/6.0; 1.0/6.0];
    beta = 1 / (alpha' * alpha);
    sigma_square = diag(Q); % compute diagonal terms only for simplicity and it is only suitable for the uncorrelated noise.
    ScaledQ = beta * sigma_square / delt;

    % First step
    w = sqrt(ScaledQ) .* randn(length(ScaledQ), 1);
    [k1, F1] = Func(x, u, w, dt, params);
    k1 = k1 * dt;

    % Second step
    w = sqrt(ScaledQ) .* randn(length(ScaledQ), 1);
    [k2, F2] = Func(x + k1 * 0.5, u, w, dt, params);
    k2 = k2 * dt;

    % Third step
    w = sqrt(ScaledQ) .* randn(length(ScaledQ), 1);
    [k3, F3] = Func(x + k2 * 0.5, u, w, dt, params);
    k3 = k3 * dt;

    % Fourth step
    w = sqrt(ScaledQ) .* randn(length(ScaledQ), 1);
    [k4, F4] = Func(x + k3, u, w, dt, params);
    k4 = k4 * dt;

    % Final update for x
    x = x + (k1 + 2 * (k2 + k3) + k4) / 6.0;

    % Calculate F_total using the same weights as for x, but without dt
    F_total = (F1 + 2 * (F2 + F3) + F4) / 6.0;
end