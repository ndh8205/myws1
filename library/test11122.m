clear all  
close all
clc


syms x y u v psi r m Fx Fy F tau Iz D dt
syms T1 T2 T3 T4 T5 T6 T7 T8


T = 1;


pos = [ x; y ];

vel = [ u; v ];

att = psi;

rate = r;

vec_state = [ pos; vel; att; rate];

disp(vec_state)

U = [T1; T2; T3; T4; T5; T6; T7; T8; tau];


B = [
        0,          0,           0,          0,           0,          0,           0,          0       0;
        0,          0,           0,          0,           0,          0,           0,          0       0;
        0,       -T/m,        -T/m,          0,           0,        T/m,         T/m,          0       0;
     -T/m,          0,           0,        T/m,         T/m,          0,           0,       -T/m       0;
        0,          0,           0,          0,           0,          0,           0,          0       0;
-(D*T)/Iz,   (D*T)/Iz,   -(D*T)/Iz,   (D*T)/Iz,   -(D*T)/Iz,   (D*T)/Iz,   -(D*T)/Iz,   (D*T)/Iz     1/Iz
     ];

disp(B)

A = [
        0,  0,    cos(psi),     -sin(psi),   0,      0;
        0,  0,    sin(psi),      cos(psi),   0,      0;
        0,  0,            0,           -r,   0,      0;
        0,  0,            r,            0,   0,      0;
        0,  0,            0,            0,   0,      1;
        0,  0,            0,            0,   0,      0 ];

disp(A)

statematrixA = A*vec_state;

disp(statematrixA)

statematrixfinal = statematrixA + B*U;

disp(statematrixfinal)

% Calculate Jacobian matrix
J = jacobian(statematrixfinal, vec_state);

% Simplify the result
% J = simplify(J);

disp('Jacobian matrix:')
disp(J)

distJ = eye(6) + A*dt;

disp('discreat matrix:')
disp(distJ)

oo = J*vec_state;
disp('vec:')
disp(oo)



