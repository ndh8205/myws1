function [x, F_total] = rk4_debug(Func, x, u, delt, params)
    % First step
    [k1, F1] = Func(x, u, params, delt);
    k1 = k1 * delt;
    
    % Second step
    [k2, F2] = Func(x + k1 * 0.5, u, params, delt);
    k2 = k2 * delt;
    
    % Third step
    [k3, F3] = Func(x + k2 * 0.5, u, params, delt);
    k3 = k3 * delt;
    
    % Fourth step
    [k4, F4] = Func(x + k3, u, params, delt);
    k4 = k4 * delt;
    
    % New state after integration
    x = x + (k1 + 2 * (k2 + k3) + k4) / 6.0;
    
    % Calculate F_total using the same weights as for x, but without delt
    F_total = (F1 + 2 * (F2 + F3) + F4) / 6.0;
end