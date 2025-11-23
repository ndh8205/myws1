clc;
clear all;
close all;

% Declare symbolic variables
syms x y u v psi r real           % State variables
syms X_t Y_t real                 % Target position

% State vector
X = [x; y; u; v; psi; r];

% Relative position in inertial frame
delta_X_I = X_t - x;
delta_Y_I = Y_t - y;
delta_pos_I = [ delta_X_I; delta_Y_I ];

% Rotation matrix from inertial to body frame (RI2B)
RI2B = [ cos(psi), sin(psi); -sin(psi), cos(psi) ];

% Transform relative position to body frame
delta_pos_B = RI2B * delta_pos_I;
delta_X_B = delta_pos_B(1);
delta_Y_B = delta_pos_B(2);

% Measurement function h(x)
rho = sqrt( delta_X_B^2 + delta_Y_B^2 );
theta = atan2( delta_Y_B, delta_X_B );

h = [ rho; theta; r ];  % Measurement vector

% Compute Jacobian H = dh/dx
H = jacobian( h, X );

% Simplify the Jacobian matrix
H = simplify(H);

% Display the measurement function and Jacobian
disp('Measurement function h(x):');
disp(h);

disp('Jacobian H:');
disp(H);
