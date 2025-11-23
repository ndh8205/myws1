function [x] = rk4(Func, x, u, delt, params)
    
    k1 = Func(x, u, params, delt) * delt;
    k2 = Func(x + k1 * 0.5, u, params, delt) * delt;
    k3 = Func(x + k2 * 0.5, u, params, delt) * delt;
    k4 = Func(x + k3, u, params, delt) * delt;
    
    x = x + (k1 + 2 * (k2 + k3) + k4) / 6.0;  % New state after integration

end