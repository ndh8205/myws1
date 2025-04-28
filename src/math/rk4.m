%% Intergral - RK4

function [x, Thrust_mass] = rk4(Func, x, u, delt, params, i)

    [k1, Thrust_mass] = Func(x, u, params, i);
    k1 = k1 * delt;

    [k2, ~] = Func(x + k1 * 0.5, u, params, i);
    k2 = k2 * delt;

    [k3, ~] = Func(x + k2 * 0.5, u, params, i);
    k3 = k3 * delt;
    
    [k4, ~] = Func(x + k3, u, params, i);
    k4 = k4 * delt;

    x = x + (k1 + 2 * (k2 + k3) + k4) / 6.0; % New state after integration

end