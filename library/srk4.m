function [x] = srk4( Func, x, u, Q, dt, params, delt )

%Scaling factor for stochastic integration
alpha = [ 1.0/6.0; 2.0/6.0; 2.0/6.0; 1.0/6.0 ];
beta = 1/( alpha' * alpha );
sigma_square = diag( Q );  %compute diagonal terms only for simplicity and it is only suituable for the uncorrelated noise. 
ScaledQ = beta * sigma_square / delt;

w = sqrt( ScaledQ ) .* randn( length( ScaledQ ),1 );
k1  = Func( x, u, w, dt, params ) * delt;

w = sqrt( ScaledQ ) .* randn( length( ScaledQ ),1 );
k2  = Func( x + k1 * 0.5 * delt, u, w, dt, params ) * delt;

w = sqrt( ScaledQ ) .* randn( length( ScaledQ ),1 );
k3  = Func( x + k2 * 0.5 * delt, u, w, dt, params ) * delt;

w = sqrt( ScaledQ ) .* randn( length( ScaledQ ),1 );
k4  = Func( x + k3 * delt, u, w, dt, params ) * delt;

x = x + ( k1 + 2 * ( k2 + k3 ) + k4 ) / 6.0;

end


