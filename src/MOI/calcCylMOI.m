function [mass, cg, inertiaTensor] = calcCylMOI(Ro,Ri,L,rho)
% 원통·원환체 질량·CG·MOI  (CG 기준)
if any([Ro Ri L rho] < 0) || Ri>Ro
    error('반지름·길이·밀도 오류'); end

V     = pi*(Ro^2 - Ri^2)*L;
mass  = rho*V;
cg    = [L/2 0 0];

Ixx   = 0.5 * mass * (Ro^2 + Ri^2);
% Iyy   = 0.25*mass*(Ro^2 + Ri^2) + (1/12)*mass*L^2;
Iyy   = 3*mass*((Ro^2 + Ri^2) + L^2)/12;
inertiaTensor = diag([Ixx Iyy Iyy]);
end
