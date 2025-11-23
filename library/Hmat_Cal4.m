clc
clear all
close all

% Symbolic variables
syms X_sat Y_sat X_target Y_target psi u v r real

% State vector
X = [ X_sat; Y_sat; u; v; psi; r ];

% Target position in inertial frame (전역 좌표계에서의 Target 위치)
X_target_vec = [ X_target; Y_target ];

% Rotation matrix from inertial to body frame (동체 좌표계로 변환)
RI2B = [ cos(psi), sin(psi); -sin(psi), cos(psi) ];

% Relative position in inertial frame (전역 좌표계에서의 상대 위치)
delta_X = X_target - X_sat;
delta_Y = Y_target - Y_sat;
delta_pos_I = [ delta_X; delta_Y ];

% Transform relative position to body frame (동체 좌표계로 변환)
delta_pos_B = RI2B * delta_pos_I;
delta_X_b = delta_pos_B(1);
delta_Y_b = delta_pos_B(2);

% Measurement function h(x)
h1 = sqrt( delta_X_b^2 + delta_Y_b^2 ); % 동체 좌표계에서의 거리
h2 = atan2( delta_Y_b, delta_X_b );     % 동체 좌표계에서의 방위각
h3 = r;                                 % 회전률 (직접 측정)
h = [ h1; h2; h3 ];

% Compute Jacobian (observation matrix)
H = jacobian(h, X);

% Simplify the Jacobian matrix
H = simplify(H);

% Display the result
disp('Observation Matrix H:')
disp(H)
