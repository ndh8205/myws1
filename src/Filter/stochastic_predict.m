function [x] = stochastic_predict( Func, x, u, Q, dt, params, i, delt )

%Scaling factor for stochastic integration
alpha = [ 1.0/6.0; 2.0/6.0; 2.0/6.0; 1.0/6.0 ];
beta = 1/( alpha' * alpha );
sigma_square = diag( Q );  %compute diagonal terms only for simplicity and it is only suituable for the uncorrelated noise. 
ScaledQ = beta * sigma_square / delt;

w = sqrt( ScaledQ ) .* randn( length( ScaledQ ),1 );
k1  = Func( x, u, w, dt, params, i ) * delt;

x = x + k1;

end