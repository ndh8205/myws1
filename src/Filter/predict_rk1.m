function [x] = predict_rk1( Func, x, u, Q, dt, params, i, delt )

n = length(x);

w = zeros(n,1);
k1  = Func( x, u, w, dt, params, i ) * delt;

x = x + k1;

end