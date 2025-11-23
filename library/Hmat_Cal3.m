clc
clear all
close all

% Symbolic variables
syms X_sat Y_sat X_ta Y_ta psi u v r rho the real

% State vector (관측자/위성의 상태)
X = [X_sat; Y_sat; u; v; psi; r];

% 관측 방정식
rho_polar_I = sqrt((X_ta - X_sat)^2 + (Y_ta - Y_sat)^2);
the_I = atan2((Y_ta - Y_sat), (X_ta - X_sat)) - psi;

% Measurement vector
h = [rho_polar_I; the_I; r];

% Compute Jacobian (observation matrix)
H = jacobian(h, X);

% Simplify the expressions
% H = simplify(H);

% Display the result
disp('Observation Matrix H:')
disp(H)
% pretty(H)

disp('H matrix elements:')
for i = 1:3
    for j = 1:6
        fprintf('H(%d,%d) = %s\n', i, j, char(H(i,j)))
    end
end