function [mass, cg, I] = calcRevMOI(x,rOut,rIn,rho)
% 회전체 (X축 회전)  – 선분별 얇은 원환체 적분
if ~isequal(size(x),size(rOut),size(rIn))
    error('길이 불일치'); end
[x,idx] = sort(x(:));
rOut = rOut(idx);  rIn = rIn(idx);

dx          = diff(x);
x_mid       = x(1:end-1) + dx/2;
rOut_mid    = (rOut(1:end-1)+rOut(2:end))/2;
rIn_mid     = (rIn (1:end-1)+rIn (2:end))/2;

dV   = pi*(rOut_mid.^2 - rIn_mid.^2).*dx;
dM   = rho*dV;
mass = sum(dM);
cg_x = sum(x_mid.*dM)/mass;
cg   = [cg_x 0 0];

Ixx_slice = 0.5 * dM .* (rOut_mid.^2 + rIn_mid.^2);
Iyy_slice = 0.25* dM .* (rOut_mid.^2 + rIn_mid.^2);
Ixx = sum(Ixx_slice);
Iyy = sum(Iyy_slice + dM.*(x_mid-cg_x).^2);
I   = diag([Ixx Iyy Iyy]);
end
