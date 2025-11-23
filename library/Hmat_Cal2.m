clear all
clc
close all

% Symbolic variables
syms X_sat Y_sat X_ta Y_ta psi u v r alpha rho the real

% Relative position in body frame
x_rel = (X_ta - X_sat) * cos(psi) + (Y_ta - Y_sat) * sin(psi);
y_rel = -(X_ta - X_sat) * sin(psi) + (Y_ta - Y_sat) * cos(psi);

% Range and bearing
rho = sqrt(x_rel^2 + y_rel^2);
the = atan2(y_rel, x_rel);

% State vector
X = [X_sat; Y_sat; u; v; psi; r];

% Observation vector
h = [rho; the];

% Compute the Jacobian (observation matrix)
H = jacobian(h, X);

% Simplify the result
H = simplify(H);

% Display the result
disp('Observation Matrix H:')
disp(H)

% If you want to see the matrix elements separately:
disp('H matrix elements:')
for i = 1:2
    for j = 1:6
        fprintf('H(%d,%d) = %s\n', i, j, char(H(i,j)))
    end
end