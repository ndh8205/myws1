function compList = buildComponentList(paramRoot)
% ========================================================================
%  rocketParams  →  compList
%  필드 : name, type, mass, cg_ref(1×3), inertia_cg(3×3)
%  ─ XML 에 기록된 치수·두께·밀도·mass 태그만 사용 (임의 기본값 無)
%  ─ 치수-or-밀도 부족하면 질량 0  + 경고 메시지 출력
%  ─ NoseCone / Transition : OpenRocket 셸 모델 적분식 사용
% ========================================================================

allC      = flattenParams(paramRoot);               % 모든 부품 1‑D 배열
compList  = struct('name',{},'type',{},'mass',{}, ...
                   'cg_ref',{},'inertia_cg',{});
lastBulkRho = NaN;                                  % 직전 bulk 재질 밀도

% ===================== 메인 루프 =====================
for k = 1:numel(allC)
    c   = allC(k);
    m   = 0; cg  = [0 0 0]; I = zeros(3);

    % --- 재질 밀도 ----------------------------------------------------
    rho = getSafe(c,'Material.DensityAttributeValue',NaN);
    if strcmp(getSafe(c,'Material.Type',''),'bulk') && ~isnan(rho)
        lastBulkRho = rho;                          % bulk 재질 기억
    end
    if isnan(rho), rho = lastBulkRho; end           % fallback

    % === 부품 타입별 계산 ============================================
    switch lower(c.Type)
    % -----------------------------------------------------------------
    case {'bodytube','tubecoupler'}
        R = getSafe(c.Geometry,{'Radius','OuterRadius'},NaN);
        t = getSafe(c.Geometry,'Thickness',NaN);
        L = getSafe(c.Geometry,'Length',NaN);
        if any(isnan([R t L rho]))
            warnSkip(c,'치수/밀도 부족');  break
        end
        [m,cg0,I] = calcCylMOI(R,R-t,L,rho);
        cg = cg0 + [c.Position.AbsoluteStartX 0 0];
    % -----------------------------------------------------------------
    case {'centeringring','bulkhead'}
        Ro = getSafe(c.Geometry,{'OuterRadius','Radius'},getSafe(c.Position,'EndRadius',NaN));
        Ri = getSafe(c.Geometry,'InnerRadius',0);
        T  = getSafe(c.Geometry,{'Thickness','Length'},NaN);
        if any(isnan([Ro T rho]))
            warnSkip(c,'치수/밀도 부족'); break
        end
        [m,~,I] = calcCylMOI(Ro,Ri,T,rho);
        cg = [c.Position.AbsoluteStartX+T/2 0 0];

    % -----------------------------------------------------------------
    case 'trapezoidfinset'
        rt = getSafe(c.Geometry,'RootChord',NaN);
        ht = getSafe(c.Geometry,'Height',NaN);
        sw = getSafe(c.Geometry,'Sweep',0);
        t  = getSafe(c.Geometry,'Thickness',NaN);
        if any(isnan([rt ht t rho]))
            warnSkip(c,'치수/밀도 부족'); break
        end
        verts = [0 0; rt 0; rt-sw ht; -sw ht];
        [m1,cg1,I1] = calcPlateMOI(verts,t,rho);
        rad = getSafe(c.Position,'EndRadius',0);
        n   = getSafe(c.InstanceInfo,'Count',1);
        rot = deg2rad(getSafe(c.InstanceInfo,'Rotation',0));
        for j = 0:n-1
            ang = j*2*pi/n + rot;
            Rz  = [cos(ang) -sin(ang) 0; sin(ang) cos(ang) 0; 0 0 1];
            cgFin = [c.Position.AbsoluteStartX+cg1(1), (rad+cg1(2))*cos(ang), (rad+cg1(2))*sin(ang)];
            compList(end+1)=struct('name',[c.Name '_' num2str(j+1)],'type','fin', ...
                     'mass',m1,'cg_ref',cgFin,'inertia_cg',Rz*I1*Rz.');
        end
        continue                           

    % -----------------------------------------------------------------
    case {'nosecone','transition'}
        L = getSafe(c.Geometry,'Length',NaN);
        t = getSafe(c.Geometry,'Thickness',NaN);
        if any(isnan([L t rho]))
            warnSkip(c,'치수/두께/밀도 부족'); break
        end
        nSlice = 200;  x = linspace(0,L,nSlice);
        shape = lower(getSafe(c.Geometry,'Shape','conical'));
        param = getSafe(c.Geometry,'ShapeParameter',0);
        if strcmpi(c.Type,'nosecone')
            Rb = getSafe(c.Geometry,'AftRadius',NaN);
            if isnan(Rb), warnSkip(c,'AftRadius 없음'); break, end
            rOut = radiusByShape(shape,x,Rb,L,param);
        else
            Rf = getSafe(c.Geometry,'ForeRadius',NaN);
            Ra = getSafe(c.Geometry,'AftRadius',NaN);
            if any(isnan([Rf Ra]))
                warnSkip(c,'Fore/Aft Radius 없음'); break
            end
            rOut = Rf + radiusByShape(shape,x,Ra-Rf,L,param);
        end
        rIn = max(rOut - t, 0);
        [m,cgLocal,I] = calcRevMOI(x,rOut,rIn,rho);
        cg = cgLocal + [c.Position.AbsoluteStartX 0 0];

    % -----------------------------------------------------------------
    case 'parachute'
        dia = getSafe(c.Geometry,'Diameter',NaN);
        if any(isnan([dia rho]))
            warnSkip(c,'직경/천 밀도 부족'); break
        end
        clothT = 3e-4;                             
        m = rho*pi*(dia/2)^2*clothT;
        nLine = getSafe(c.Geometry.ParachuteDetails,'LineCount',0);
        LenL  = getSafe(c.Geometry.ParachuteDetails,'LineLength',0);
        rhoL  = getSafe(c.Geometry.ParachuteDetails.LineMaterial,'DensityAttributeValue',0);
        m = m + nLine*LenL*rhoL;
        Lp = getSafe(c.Geometry,'PackedLength',0);
        cg = [c.Position.AbsoluteStartX+Lp/2 0 0];

    % -----------------------------------------------------------------
    case 'masscomponent'
        m  = getSafe(c,'Mass',0);
        Lp = getSafe(c.Geometry,'PackedLength',0);
        cg = [c.Position.AbsoluteStartX+Lp/2 0 0];

    otherwise
        warnSkip(c,'지원 안함');
    end

    compList(end+1)=struct('name',c.Name,'type',c.Type,'mass',m, ...
               'cg_ref',cg,'inertia_cg',I);
end
% =====================================================
end   % buildComponentList

%% ===================== 헬퍼 함수들 ====================
function v = getSafe(S,field,def)
% 구조체 S 에서 'a.b.c' 또는 {'A','B'} 경로의 값을 안전하게 추출
v = def;
if isempty(S), return, end
if iscell(field)
    for f = field
        tmp = getSafe(S,f{1},NaN);
        if ~isnan(tmp), v = tmp; return, end
    end
    return
end
parts = strsplit(char(field),'.'); tmp = S;
for p = parts
    if isstruct(tmp) && isfield(tmp,p{1}), tmp = tmp.(p{1});
    else, return, end
end
v = tmp;
end

function warnSkip(c,msg)
fprintf(' "%s" (%s) %s → 0 kg 처리\n',c.Name,c.Type,msg);
end

%% ------------ 원통 셸 MOI --------------------------------------------
function [m,cg,Icg] = calcCylMOI(Ro,Ri,L,rho)
V = pi*(Ro^2 - Ri^2)*L; m = rho*V;
cg = [L/2 0 0];
Ixx = 0.5*m*(Ro^2 + Ri^2);
Iyy = m*((Ro^2 + Ri^2)/4 + L^2/12);
Icg = diag([Ixx,Iyy,Iyy]);
end

%% ------------ 평판 핀 MOI --------------------------------------------
function [m,cg,Icg] = calcPlateMOI(verts,t,rho)
[A,cent,I2] = polygeom(verts(:,1),verts(:,2));
m = A*t*rho; cg = [cent 0];
Izz = I2*t*rho; Iyy = m*cent(1)^2 + Izz; Icg = diag([Iyy,Iyy,Izz]);
end

%% ------------ 회전체 셸 MOI -----------------------------------------
function [m,cg,Icg] = calcRevMOI(x,rOut,rIn,rho)
A = pi*(rOut.^2 - rIn.^2); V = trapz(x,A); m = rho*V;
Mx = trapz(x,x.*A); xcg = Mx/V; cg = [xcg 0 0];
Iroll = rho*trapz(x,0.5*pi*(rOut.^4 - rIn.^4));
Iyy   = rho*trapz(x,pi*(rOut.^2 - rIn.^2).*(x-xcg).^2);
Icg   = diag([Iroll,Iyy,Iyy]);
end

%% ----------- Transition.Shape 방정식 -------------------------------
function r = radiusByShape(shape,x,R,L,param)
switch shape
    case 'conical',       r = R.*x./L;
    case 'ogive'
        if param==0, r = R.*x./L; return, end
        Lp = L/param; R0 = sqrt(((L^2+R^2)*((2-param)*L)^2+(param*R)^2)/(4*(param*R)^2));
        y0 = sqrt(R0^2 - Lp^2);
        r  = sqrt(R0^2 - (Lp - x).^2) - y0;
    case 'ellipsoid',     r = sqrt(2*R.*x - x.^2).*(R/L);
    case 'power',         r = R.*(x./L).^param;
    case 'parabolicseries', r = R.*((2.*x./L - param.*(x./L).^2)./(2-param));
    case 'haack'
        theta = acos(1-2*x./L);
        if param==0
            r = R.*sqrt((theta - sin(2*theta)/2)./pi);
        else
            r = R.*sqrt((theta - sin(2*theta)/2 + param.*sin(theta).^3)./pi);
        end
    otherwise,            r = R.*x./L;
end
end

%% ------------- polygeom (Standalone) ---------------------------------
function [A,cent,Izz] = polygeom(x,y)
% 계산: 다각형 면적, 무게중심, z-축 2차 모멘트 (about centroid)
% 입력: x, y  - 꼭짓점 배열 (마지막점 = 첫점 아니어도 됨)
    x = x(:); y = y(:);
    if x(1)~=x(end) || y(1)~=y(end)
        x(end+1)=x(1); y(end+1)=y(1);
    end
    xi = x(1:end-1);  yi = y(1:end-1);
    xi1= x(2:end);    yi1= y(2:end);
    cross = xi.*yi1 - xi1.*yi;
    A = 0.5*sum(cross);
    if abs(A)<eps,  A = 0; cent=[0 0]; Izz=0; return, end
    Cx = (1/(6*A))*sum((xi+xi1).*cross);
    Cy = (1/(6*A))*sum((yi+yi1).*cross);
    cent = [Cx Cy];
    Iz0 = (1/12)*sum(cross.*(xi.^2 + xi.*xi1 + xi1.^2 + yi.^2 + yi.*yi1 + yi1.^2));
    Izz = Iz0 - A*(Cx^2 + Cy^2);
end