function [mass, cg, I] = calcPlateMOI(v,t,rho)
% v : Nx2 꼭짓점, 판은 XY 평면, z-두께 t
if size(v,2)~=2 || size(v,1)<3 || any(t<0) || rho<0
    error('calcPlateMOI 입력 오류'); end

v_closed   = [v; v(1,:)];
n          = size(v,1);

% ① 면적 & CG
A=0; Cx=0; Cy=0;
for k = 1:n
    cross = v_closed(k,1)*v_closed(k+1,2)-v_closed(k+1,1)*v_closed(k,2);
    A  = A  + cross;
    Cx = Cx + (v_closed(k,1)+v_closed(k+1,1))*cross;
    Cy = Cy + (v_closed(k,2)+v_closed(k+1,2))*cross;
end
A  = A/2;
Cx = Cx/(6*A);  Cy = Cy/(6*A);
cg = [Cx Cy t/2];

V     = A*t;
mass  = rho*V;

w = max(v(:,1))-min(v(:,1));
h = max(v(:,2))-min(v(:,2));
Ixx = (1/12)*mass*(h^2 + t^2);
Iyy = (1/12)*mass*(w^2 + t^2);
Izz = (1/12)*mass*(w^2 + h^2);
I   = diag([Ixx Iyy Izz]);
end
