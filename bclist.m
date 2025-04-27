function compList = bclist(flatRocketParams)
% bclist  로켓 부품 평탄 리스트에서 질량, 무게중심, 관성모멘트 리스트 작성
%
% compList = bclist(flatRocketParams)
%
% 입력
% flatRocketParams : flattenRocketStructure로 얻은 평탄화된 로켓 파라미터 구조체
%
% 출력
% compList : 각 부품별 질량특성 정보를 담은 구조체 배열
% ├─ name : 부품 이름
% ├─ type : 부품 유형
% ├─ mass : 질량 (kg)
% ├─ cg_ref : 기준좌표계 기준 무게중심 [x,y,z]
% └─ inertia_cg : 무게중심 기준 관성텐서 (3x3)
%
% ========================================================================
%  flatRocketParams  →  compList
%  필드 : name, type, mass, cg_ref(1×3), inertia_cg(3×3)
%  ─ XML 에 기록된 치수·두께·밀도·mass 태그만 사용 (임의 기본값 無)
%  ─ 치수-or-밀도 부족하면 질량 0  + 경고 메시지 출력
%  ─ NoseCone / Transition : OpenRocket 셸 모델 적분식 사용
% ========================================================================

% 결과 저장을 위한 구조체 배열 초기화
compList = struct('name',{},'type',{},'mass',{}, ...
                 'cg_ref',{},'inertia_cg',{});

% 입력이 비어있는 경우 빈 결과 반환
if isempty(flatRocketParams)
    fprintf('경고: 입력된 평탄화 로켓 구조체가 비어있습니다.\n');
    return;
end

fprintf('\n--- 질량 특성 계산 시작 ---\n');
lastBulkRho = NaN;  % 직전 bulk 재질 밀도 저장용

% ===================== 메인 루프 =====================
for k = 1:numel(flatRocketParams)
    c = flatRocketParams(k);
    m = 0; cg = [0 0 0]; I = zeros(3);
    
    % --- 재질 밀도 ----------------------------------------------------
    % 먼저 Material 필드 확인, 없으면 Matrial(오타 필드) 확인
    if isfield(c, 'Material') && ~isempty(c.Material)
        material = c.Material;
    elseif isfield(c, 'Matrial') && ~isempty(c.Matrial)
        material = c.Matrial;
    else
        material = struct();
    end
    
    rho = getSafe(material, 'DensityAttributeValue', NaN);
    materialType = getSafe(material, 'Type', '');
    if strcmpi(materialType, 'bulk') && ~isnan(rho)
        lastBulkRho = rho;  % bulk 재질 기억
    end
    if isnan(rho), rho = lastBulkRho; end  % fallback

    % === 부품 타입별 계산 ============================================
    switch lower(getSafe(c, 'Type', ''))
    % -----------------------------------------------------------------
    case {'bodytube','tubecoupler'}
        R = getSafe(c.Geometry, {'Radius','OuterRadius'}, NaN);
        t = getSafe(c.Geometry, 'Thickness', NaN);
        L = getSafe(c.Geometry, 'Length', NaN);
        if any(isnan([R t L rho]))
            warnSkip(c, '치수/밀도 부족');  
            break
        end
        [m, cg0, I] = calcCylMOI(R, R-t, L, rho);
        cg = cg0 + [c.Position.AbsoluteStartX 0 0];
    % -----------------------------------------------------------------
    case {'centeringring','bulkhead'}
        Ro = getSafe(c.Geometry, {'OuterRadius','Radius'}, getSafe(c.Position, 'EndRadius', NaN));
        Ri = getSafe(c.Geometry, 'InnerRadius', 0);
        T = getSafe(c.Geometry, {'Thickness','Length'}, NaN);
        if any(isnan([Ro T rho]))
            warnSkip(c, '치수/밀도 부족'); 
            break
        end
        [m, ~, I] = calcCylMOI(Ro, Ri, T, rho);
        cg = [c.Position.AbsoluteStartX+T/2 0 0];
    % -----------------------------------------------------------------
    case 'nosecone'
        L = getSafe(c.Geometry, 'Length', NaN);
        R = getSafe(c.Geometry, 'AftRadius', NaN);
        t = getSafe(c.Geometry, 'Thickness', NaN);
        if any(isnan([L R t rho]))
            warnSkip(c, '치수/두께/밀도 부족'); 
            break
        end
        
        % 형상 정보 가져오기
        shape = lower(getSafe(c.Geometry, 'Shape', 'conical'));
        param = getSafe(c.Geometry, 'ShapeParameter', 0);
        
        % 적분 포인트 생성
        nSlice = 200;  
        x = linspace(0, L, nSlice);
        
        % 반경 프로파일 계산
        rOut = radiusByShape(shape, x, R, L, param);
        rIn = max(rOut - t, 0);  % 내부 반경 (두께 고려)
        
        % 질량 특성 계산
        [m, cgLocal, I] = calcRevMOI(x, rOut, rIn, rho);
        cg = cgLocal + [c.Position.AbsoluteStartX 0 0];
        
        % 어깨 부분 추가 계산 (있는 경우)
        shoulderLength = getSafe(c.Geometry, 'AftShoulderLength', 0);
        shoulderRadius = getSafe(c.Geometry, 'AftShoulderRadius', 0);
        
        if shoulderLength > 0 && shoulderRadius > 0
            % 어깨 부분은 간단한 원통으로 계산
            [mShoulder, cgShoulder0, IShoulder] = calcCylMOI(shoulderRadius, shoulderRadius-t, shoulderLength, rho);
            cgShoulder = cgShoulder0 + [L 0 0];  % 어깨 시작점은 노즈콘 끝
            
            % 질량 중심 통합
            totalMass = m + mShoulder;
            if totalMass > 0
                cg = (m*cg + mShoulder*[c.Position.AbsoluteStartX+cgShoulder(1) 0 0]) / totalMass;
                I = I + IShoulder;  % 간단한 관성텐서 합산 (근사)
                m = totalMass;
            end
        end
    % -----------------------------------------------------------------
    case 'transition'
        L = getSafe(c.Geometry, 'Length', NaN);
        Rf = getSafe(c.Geometry, 'ForeRadius', NaN);
        Ra = getSafe(c.Geometry, 'AftRadius', NaN);
        t = getSafe(c.Geometry, 'Thickness', NaN);
        
        if any(isnan([L Rf Ra t rho]))
            warnSkip(c, 'Transition 치수/두께/밀도 부족'); 
            break
        end
        
        % 형상 정보 가져오기
        shape = lower(getSafe(c.Geometry, 'Shape', 'conical'));
        
        % 적분 포인트 생성
        nSlice = 200;  
        x = linspace(0, L, nSlice);
        
        % Transition 형상일 경우 반경 프로파일
        rOut = Rf + radiusByShape(shape, x, Ra-Rf, L, 0);
        rIn = max(rOut - t, 0);  % 내부 반경 (두께 고려)
        
        % 질량 특성 계산
        [m, cgLocal, I] = calcRevMOI(x, rOut, rIn, rho);
        cg = cgLocal + [c.Position.AbsoluteStartX 0 0];
    % -----------------------------------------------------------------
    case 'trapezoidfinset'
        % 사다리꼴 핀 치수 데이터 추출
        rt = getSafe(c.Geometry, 'RootChord', NaN);    % 루트 코드 길이
        tp = getSafe(c.Geometry, 'TipChord', 0);       % 팁 코드 길이
        ht = getSafe(c.Geometry, 'Height', NaN);       % 높이
        sw = getSafe(c.Geometry, 'Sweep', 0);          % 스윕
        t = getSafe(c.Geometry, 'Thickness', NaN);     % 두께
        
        % 캔트 각도 (각도 -> 라디안)
        cantAngle = deg2rad(getSafe(c, 'CantAngle', 0));
        
        % 단면 형태 (기본값: SQUARE)
        crossSection = getSafe(c, 'CrossSection', 'SQUARE');
        crossSectionFactor = 1.0; % 기본값
        if strcmpi(crossSection, 'ROUNDED')
            crossSectionFactor = 0.99;
        elseif strcmpi(crossSection, 'AIRFOIL')
            crossSectionFactor = 0.85;
        end
        
        % 탭 관련 정보
        tabHeight = getSafe(c, 'TabHeight', 0);
        tabLength = getSafe(c, 'TabLength', 0);
        tabPosition = getSafe(c, 'TabPosition', 0);
        
        % 필렛 관련 정보
        filletRadius = getSafe(c, 'FilletRadius', 0);
        filletDensity = getSafe(c, 'FilletMaterial.DensityAttributeValue', rho);
        
        if any(isnan([rt ht t rho]))
            warnSkip(c, '치수/밀도 부족'); 
            break
        end
        
        % 사다리꼴 핀 꼭지점 좌표 계산 - TipChord 고려
        if tp > 0
            verts = [0 0; rt 0; rt-sw+tp ht; -sw ht];
        else
            verts = [0 0; rt 0; rt-sw ht; -sw ht];
        end
        
        % 핀 단일 측면의 면적, 무게중심, 관성 계산
        [m1, cg1, I1] = calcPlateMOI(verts, t, rho);
        
        % 단면 형태를 고려한 부피 및 질량 조정
        m1 = m1 * crossSectionFactor;
        
        % 탭 계산
        mTab = 0;
        cgTab = [0 0 0];
        ITab = zeros(3);
        
        if tabHeight > 0 && tabLength > 0
            % 탭 기하학적 정의
            tabVerts = [tabPosition -tabHeight; 
                       tabPosition 0; 
                       tabPosition+tabLength 0; 
                       tabPosition+tabLength -tabHeight];
            
            % 탭 질량과 무게중심, 관성 계산
            [mTab, cgTab0, ITab] = calcPlateMOI(tabVerts, t, rho);
            cgTab = [cgTab0(1), cgTab0(2), 0];  % 3D 벡터로 확장
            mTab = mTab * crossSectionFactor;  % 단면 형태 고려
        end
        
        % 필렛 계산
        mFillet = 0;
        cgFillet = [0 0 0];
        IFillet = zeros(3);
        
        if filletRadius > 0
            % 대략적인 필렛 단면적 계산
            filletArea = 0.5 * filletRadius * filletRadius;
            filletLength = rt; % 루트 코드 길이를 필렛 길이로 사용
            
            mFillet = filletArea * filletLength * filletDensity;
            cgFillet = [rt/2, filletRadius/3, 0];  % 필렛 CG 좌표
            
            % 필렛의 관성 텐서 (대략적 계산)
            IFillet = diag([
                mFillet*filletRadius*filletRadius/12, 
                mFillet*rt*rt/12, 
                mFillet*(filletRadius*filletRadius+rt*rt)/12
            ]);
        end
        
        % 총 질량 및 무게중심 계산 (핀+탭+필렛)
        mTotal = m1 + mTab + mFillet;
        
        % CG 계산을 위한 3D 좌표 준비
        cg1_3D = [cg1(1), cg1(2), 0];  % 2D를 3D로 확장
        
        % 질량 가중 평균으로 무게중심 계산
        if mTotal > 0
            cgFin1 = (m1*cg1_3D + mTab*cgTab + mFillet*cgFillet) / mTotal;
        else
            cgFin1 = cg1_3D;
        end
        
        % 총 관성 텐서 계산 (단순 합산)
        I_total = I1 + ITab + IFillet;
        
        % 핀셋 정보 (절대 좌표계 기준)
        % 몸체 반경 및 설치 정보
        rad = getSafe(c.Position, 'EndRadius', 0);       % 부모 본체의 반경
        n = getSafe(c.InstanceInfo, 'Count', 1);         % 핀 개수
        rot = deg2rad(getSafe(c.InstanceInfo, 'Rotation', 0));  % 첫 번째 핀 회전 오프셋
        xStart = getSafe(c.Position, 'AbsoluteStartX', 0);     % 핀셋의 절대 X 시작 위치
        
        % 각 핀에 대한 처리
        for j = 0:n-1
            % 핀 배치 각도 (균등 분포)
            ang = j * (2*pi/n) + rot;  % j번째 핀의 각도
            
            % 핀 위치 (로컬 좌표계, 핀 기준점 기준)
            xCG_local = cgFin1(1);      % 핀 CG의 X 좌표 (루트 코드 앞쪽 기준)
            yCG_local = cgFin1(2);      % 핀 CG의 Y 좌표 (본체 표면에서 수직 거리)
            
            % 본체 반경 (핀 부착 위치)
            bodyRadius = rad;
            
            % 핀 부착 방향 벡터 계산 (캔트 적용 전, 단위 벡터)
            baseVector = [0, cos(ang), sin(ang)];  % YZ 평면의 방향 벡터
            
            % 캔트 각도 적용 (필요시)
            if abs(cantAngle) > 1e-6
                % Y축 기준 회전 행렬 (X-Z 평면에서 회전)
                Ry = [
                    cos(cantAngle), 0, sin(cantAngle);
                    0, 1, 0;
                    -sin(cantAngle), 0, cos(cantAngle)
                ];
                
                % 방향 벡터에 캔트 적용
                baseVector = (baseVector * Ry')';  % 행벡터 × 회전행렬
            end
            
            % 최종 CG 위치 계산
            % 1. 절대 X 위치 = 핀셋 시작 X + 핀 로컬 X CG
            cgX = xStart + xCG_local;
            
            % 2. 반경 방향 위치 = 본체 반경 + 핀 CG까지의 거리
            totalRadius = bodyRadius + yCG_local;
            
            % 3. Y, Z 좌표 계산 (방향 벡터 × 거리)
            cgY = totalRadius * baseVector(2);  % Y 성분
            cgZ = totalRadius * baseVector(3);  % Z 성분
            
            % 최종 무게중심 위치
            cgFin = [cgX, cgY, cgZ];
            
            % 관성 텐서 회전
            % 1. 회전 행렬 계산 (핀 기본 방향 → 설치 방향)
            R = eye(3);  % 기본 단위 행렬
            
            % 2. 캔트 회전 적용 (X-Z 평면 회전)
            if abs(cantAngle) > 1e-6
                Ry = [
                    cos(cantAngle), 0, sin(cantAngle);
                    0, 1, 0;
                    -sin(cantAngle), 0, cos(cantAngle)
                ];
                R = R * Ry;  % 회전 행렬 업데이트
            end
            
            % 3. 원주 방향 회전 적용 (YZ 평면)
            Rz = [
                1, 0, 0;
                0, cos(ang), -sin(ang);
                0, sin(ang), cos(ang)
            ];
            R = Rz * R;  % 최종 회전 행렬
            
            % 관성 텐서 회전 변환
            I_rotated = R * I_total * R';
            
            % 핀 이름 생성 (인덱스 기반)
            finName = sprintf('%s_%d', getSafe(c, 'Name', ['Fin_' num2str(k)]), j+1);
            
            % 부품 목록에 추가
            compList(end+1) = struct('name', finName, 'type', 'fin', ...
                                   'mass', mTotal, 'cg_ref', cgFin, 'inertia_cg', I_rotated);
        end
        
        continue
        
    % -----------------------------------------------------------------
    case 'parachute'
        dia = getSafe(c.Geometry, 'Diameter', NaN);
        if any(isnan([dia rho]))
            warnSkip(c, '직경/천 밀도 부족'); 
            break
        end
        clothT = 3e-4;                             
        m = rho*pi*(dia/2)^2*clothT;
        nLine = getSafe(c.Geometry.ParachuteDetails, 'LineCount', 0);
        LenL = getSafe(c.Geometry.ParachuteDetails, 'LineLength', 0);
        rhoL = getSafe(c.Geometry.ParachuteDetails.LineMaterial, 'DensityAttributeValue', 0);
        m = m + nLine*LenL*rhoL;
        Lp = getSafe(c.Geometry, 'PackedLength', 0);
        cg = [c.Position.AbsoluteStartX+Lp/2 0 0];
    % -----------------------------------------------------------------
    case 'masscomponent'
        m = getSafe(c, 'Mass', 0);
        Lp = getSafe(c.Geometry, 'PackedLength', 0);
        cg = [c.Position.AbsoluteStartX+Lp/2 0 0];
    % -----------------------------------------------------------------
    otherwise
        warnSkip(c, '지원 안함');
        continue;
    end

    compList(end+1) = struct('name', getSafe(c, 'Name', ['Component_' num2str(k)]), ...
                            'type', getSafe(c, 'Type', 'unknown'), ...
                            'mass', m, 'cg_ref', cg, 'inertia_cg', I);
end

fprintf('질량 특성 계산 완료: 총 %d개 부품 처리됨\n', numel(compList));
% =====================================================
end   % bclist

%% ===================== 헬퍼 함수들 ====================
function v = getSafe(S, field, def)
% 구조체 S 에서 'a.b.c' 또는 {'A','B'} 경로의 값을 안전하게 추출
v = def;
if isempty(S) || ~isstruct(S), return, end

if iscell(field)
    for f = field
        if ischar(f{1}) || isstring(f{1})
            tmp = getSafe(S, f{1}, NaN);
            if ~isnan(tmp), v = tmp; return, end
        end
    end
    return
end

if ~(ischar(field) || isstring(field)), return, end

parts = strsplit(char(field), '.'); 
tmp = S;
for p = parts
    if isstruct(tmp) && isfield(tmp, p{1})
        tmp = tmp.(p{1});
    else
        return
    end
end
v = tmp;
end

function warnSkip(c, msg)
fprintf(' "%s" (%s) %s → 0 kg 처리\n', getSafe(c, 'Name', 'Unknown'), getSafe(c, 'Type', 'Unknown'), msg);
end

%% ------------ 원통 셸 MOI --------------------------------------------
function [m, cg, Icg] = calcCylMOI(Ro, Ri, L, rho)
V = pi*(Ro^2 - Ri^2)*L; 
m = rho*V;
cg = [L/2 0 0];
Ixx = 0.5*m*(Ro^2 + Ri^2);
Iyy = m*((Ro^2 + Ri^2)/4 + L^2/12);
Icg = diag([Ixx, Iyy, Iyy]);
end

%% ------------ 평판 핀 MOI --------------------------------------------
function [m, cg, Icg] = calcPlateMOI(verts, t, rho)
[A, cent, I2] = polygeom(verts(:,1), verts(:,2));
m = A*t*rho; 
cg = cent;  % 2D 좌표 반환
Izz = I2*t*rho; 
Iyy = m*cent(1)^2 + Izz; 
Icg = diag([Iyy, Iyy, Izz]);
end

%% ------------ 회전체 셸 MOI -----------------------------------------
function [m, cg, Icg] = calcRevMOI(x, rOut, rIn, rho)
A = pi*(rOut.^2 - rIn.^2); 
V = trapz(x, A); 
m = rho*V;
Mx = trapz(x, x.*A); 
xcg = Mx/V; 
cg = [xcg 0 0];
Iroll = rho*trapz(x, 0.5*pi*(rOut.^4 - rIn.^4));
Iyy = rho*trapz(x, pi*(rOut.^2 - rIn.^2).*(x-xcg).^2);
Icg = diag([Iroll, Iyy, Iyy]);
end

%% ----------- Transition.Shape 방정식 -------------------------------
function r = radiusByShape(shape, x, R, L, param)
switch shape
    case 'conical',       r = R.*x./L;
    case 'ogive'
        if param==0, r = R.*x./L; return, end
        Lp = L/param; R0 = sqrt(((L^2+R^2)*((2-param)*L)^2+(param*R)^2)/(4*(param*R)^2));
        y0 = sqrt(R0^2 - Lp^2);
        r = sqrt(R0^2 - (Lp - x).^2) - y0;
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
function [A, cent, Izz] = polygeom(x, y)
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