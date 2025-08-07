function compList_static = static_param(flatRocketParams)
% static_param  모터를 제외한 로켓 정적 부품의 질량 특성 계산
%
% compList_static = static_param(flatRocketParams)
%
% 입력
% flatRocketParams : flattenRocketStructure로 얻은 평탄화된 로켓 파라미터 구조체
%
% 출력
% compList_static : 모터를 제외한 각 부품별 질량특성 정보를 담은 구조체 배열
% ├─ name : 부품 이름
% ├─ type : 부품 유형
% ├─ mass : 질량 (kg)
% ├─ cg_ref : 기준좌표계 기준 무게중심 [x,y,z]
% └─ inertia_cg : 무게중심 기준 관성텐서 (3x3)

% 결과 저장을 위한 구조체 배열 초기화
compList_static = struct('name',{},'type',{},'mass',{}, ...
                 'cg_ref',{},'inertia_cg',{});

% 입력이 비어있는 경우 빈 결과 반환
if isempty(flatRocketParams)
    fprintf('경고: 입력된 평탄화 로켓 구조체가 비어있습니다.\n');
    return;
end

fprintf('\n--- 정적 부품 질량 특성 계산 시작 ---\n');
lastBulkRho = NaN;  % 직전 bulk 재질 밀도 저장용
lastLineRho = NaN;  % 직전 낙하산 줄 밀도 저장용

% 핀셋 인덱스 추적
finSetIndices = [];

% ===================== 메인 루프 =====================
for k = 1:numel(flatRocketParams)
    c = flatRocketParams(k);
    m = 0; cg = [0 0 0]; I = zeros(3);
    
    % 모터 타입이면 건너뛰기
    if strcmpi(getSafe(c, 'Type', ''), 'motor')
        fprintf('정보: 모터 "%s"는 정적 부품 목록에서 제외됩니다.\n', ...
                getSafe(c, 'Name', 'Unknown Motor'));
        continue;
    end
    
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
        t = getSafe(c.Geometry, 'Thickness', NaN); % 주 형상 두께

        if any(isnan([L Rf Ra t rho]))
            warnSkip(c, 'Transition 치수/두께/밀도 부족');
            continue; % break 대신 continue 사용
        end

        % 형상 정보 가져오기
        shape = lower(getSafe(c.Geometry, 'Shape', 'conical'));
        param = getSafe(c.Geometry, 'ShapeParameter', 0); % 실제 param 값 사용

        % 적분 포인트 생성
        nSlice = 200;
        x = linspace(0, L, nSlice);

        % Transition 형상일 경우 반경 프로파일 계산
        shape_factor = radiusByShape(shape, x, 1, L, param);
        rOut = Rf + (Ra - Rf) * shape_factor;
        rIn = max(rOut - t, 0);  % 내부 반경 (두께 고려)

        % 질량 특성 계산 (주 형상)
        [mBody, cgLocalBody, IBody] = calcRevMOI(x, rOut, rIn, rho);
        cgBodyAbs = cgLocalBody + [c.Position.AbsoluteStartX 0 0]; % 주 형상의 절대 CG

        % --- 수정 시작: Transition 어깨 질량 계산 추가 ---
        mForeShoulder = 0; mAftShoulder = 0;
        cgForeShoulderAbs = [NaN NaN NaN]; cgAftShoulderAbs = [NaN NaN NaN]; % 초기값을 NaN으로 하여 계산 여부 확인

        % 앞쪽 어깨 (Fore Shoulder)
        foreShoulderL = getSafe(c.Geometry, 'ForeShoulderLength', 0);
        foreShoulderR = getSafe(c.Geometry, 'ForeShoulderRadius', 0);
        % 어깨 두께 필드가 없으면 주 형상 두께(t) 사용
        foreShoulderT = getSafe(c.Geometry, 'ForeShoulderThickness', t);
        if foreShoulderL > 0 && foreShoulderR > 0 && ~isnan(foreShoulderT)
            % 앞쪽 어깨는 Transition 시작점(StartX)의 *앞*에 위치함
            [mForeShoulder, cgForeShoulderLocal, ~] = calcCylMOI(foreShoulderR, max(0, foreShoulderR - foreShoulderT), foreShoulderL, rho);
            cgForeShoulderAbs = [c.Position.AbsoluteStartX - foreShoulderL + cgForeShoulderLocal(1), 0, 0];
        end

        % 뒤쪽 어깨 (Aft Shoulder)
        aftShoulderL = getSafe(c.Geometry, 'AftShoulderLength', 0);
        aftShoulderR = getSafe(c.Geometry, 'AftShoulderRadius', 0);
        % 어깨 두께 필드가 없으면 주 형상 두께(t) 사용
        aftShoulderT = getSafe(c.Geometry, 'AftShoulderThickness', t);
        if aftShoulderL > 0 && aftShoulderR > 0 && ~isnan(aftShoulderT)
            % 뒤쪽 어깨는 Transition 끝점(StartX + L)의 *뒤*에 위치함
            [mAftShoulder, cgAftShoulderLocal, ~] = calcCylMOI(aftShoulderR, max(0, aftShoulderR - aftShoulderT), aftShoulderL, rho);
            cgAftShoulderAbs = [c.Position.AbsoluteStartX + L + cgAftShoulderLocal(1), 0, 0];
        end

        % 질량 및 CG 통합
        m = mBody + mForeShoulder + mAftShoulder; % 총 질량

        % CG 계산: 각 부분의 질량이 0보다 클 때만 가중 평균에 포함
        weightedCgSum = [0 0 0];
        if mBody > 1e-9, weightedCgSum = weightedCgSum + mBody * cgBodyAbs; end
        if mForeShoulder > 1e-9, weightedCgSum = weightedCgSum + mForeShoulder * cgForeShoulderAbs; end
        if mAftShoulder > 1e-9, weightedCgSum = weightedCgSum + mAftShoulder * cgAftShoulderAbs; end

        if m > 1e-9 % 총 질량이 0보다 클 때만 CG 계산
            cg = weightedCgSum / m;
        else
            cg = cgBodyAbs; % 질량 없으면 주 형상 CG (절대 좌표)
            m = 0;
        end

        % MOI 저장 (주 형상만 저장 - 수정 없음)
        I = IBody;
        % --- Transition 어깨 질량 계산 추가 끝 ---
    % -----------------------------------------------------------------
    case 'trapezoidfinset'
        % 핀셋 인덱스 저장
        finSetIndices(end+1) = k;
        
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
        
        % 필렛 계산 - OpenRocket 방식으로 수정
        mFillet = 0;
        cgFillet = [0 0 0];
        IFillet = zeros(3);
        
        if filletRadius > 0
            % 몸체 반경 (핀 부착 위치)
            bodyRadius = getSafe(c.Position, 'EndRadius', 0);
            
            % 필렛 단면적 계산 (OpenRocket 방식)
            % 필렛 세그먼트 갯수 설정 (더 정교한 계산을 위해 세그먼트 수 증가)
            nSegments = 200;
            segmentLength = rt / nSegments;
            
            totalFilletVolume = 0;
            totalFilletMass = 0;
            weightedCGSum = [0, 0, 0];
            
            for segIdx = 1:nSegments
                % 현재 세그먼트의 x 좌표 (핀 루트 기준)
                xSegment = (segIdx - 0.5) * segmentLength;
                
                % OpenRocket의 필렛 단면적 계산 공식 적용
                hypotenuse = filletRadius + bodyRadius;
                
                % NaN 방지 처리
                if hypotenuse <= 0
                    continue;
                end
                
                innerArcAngle = asin(min(filletRadius / hypotenuse, 1.0));
                outerArcAngle = acos(min(filletRadius / hypotenuse, 1.0));
                
                triangleArea = tan(outerArcAngle) * filletRadius * filletRadius / 2;
                segmentFilletArea = triangleArea - outerArcAngle * filletRadius * filletRadius / 2 - innerArcAngle * bodyRadius * bodyRadius / 2;
                
                % NaN 확인
                if isnan(segmentFilletArea)
                    segmentFilletArea = 0;
                else
                    % 각 핀은 양쪽에 필렛이 있음
                    segmentFilletArea = segmentFilletArea * 2;
                end
                
                % 세그먼트 부피 및 질량
                segmentVolume = segmentFilletArea * segmentLength;
                segmentMass = segmentVolume * filletDensity;
                
                % 세그먼트 무게중심 (OpenRocket 공식 사용)
                % yCentroid는 본체 중심에서 약 bodyRadius + filletRadius/5 위치
                yCentroid = bodyRadius + filletRadius / 5;
                segmentCG = [xSegment, yCentroid, 0];
                
                % 총합 계산
                totalFilletVolume = totalFilletVolume + segmentVolume;
                totalFilletMass = totalFilletMass + segmentMass;
                weightedCGSum = weightedCGSum + segmentMass * segmentCG;
            end
            
            % 최종 필렛 질량 및 무게중심
            mFillet = totalFilletMass;
            
            if mFillet > 0
                cgFillet = weightedCGSum / mFillet;
            else
                cgFillet = [rt/2, bodyRadius + filletRadius/5, 0];
            end
            
            % 필렛의 관성 텐서 (간략화된 계산 유지)
            IFillet = diag([
                mFillet * filletRadius * filletRadius / 12, 
                mFillet * rt * rt / 12, 
                mFillet * (filletRadius * filletRadius + rt * rt) / 12
            ]);
        end
        
        % 총 질량 및 무게중심 계산 (핀+탭+필렛)
        mTotal = m1 + mTab + mFillet;
        
        % CG 계산을 위한 3D 좌표 준비
        cg1_3D = [cg1(1), cg1(2), 0];  % 2D를 3D로 확장
        
        % 질량 가중 평균으로 무게중심 계산
        if mTotal > 0
            cgFin1 = (m1 * cg1_3D + mTab * cgTab + mFillet * cgFillet) / mTotal;
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
            compList_static(end+1) = struct('name', finName, 'type', 'fin', ...
                                   'mass', mTotal, 'cg_ref', cgFin, 'inertia_cg', I_rotated);
        end
        
        continue
        
    % -----------------------------------------------------------------
    case 'parachute'
        dia = getSafe(c.Geometry, 'Diameter', NaN);
        % rho는 루프 시작 시 결정됨 (표면 밀도 또는 fallback된 부피 밀도)

        if isnan(dia) || isnan(rho) % 천 밀도 확인
            warnSkip(c, '직경 또는 천 재질 밀도(Surface/Bulk) 부족');
            continue;
        end

        % --- 수정: rhoL 결정 로직을 사용 전에 위치 ---
        rhoL = NaN; % 매번 초기화
        if isfield(c, 'Geometry') && isfield(c.Geometry, 'ParachuteDetails') ...
                && isfield(c.Geometry.ParachuteDetails, 'LineMaterial')
             rhoL_local = getSafe(c.Geometry.ParachuteDetails.LineMaterial, 'DensityAttributeValue', NaN);
             if ~isnan(rhoL_local)
                 rhoL = rhoL_local; % 개별 라인 재질 밀도 사용
                 % lastLineRho 업데이트는 루프 시작 시 수행됨
             end
        end
        if isnan(rhoL) % 개별 설정 없으면 fallback
            rhoL = lastLineRho;
        end
        if isnan(rhoL) % fallback 실패 시 0으로 설정 및 경고
            rhoL = 0;
             fprintf('경고: %s (%s) - 낙하산 줄 밀도 정보가 없어 0으로 처리됩니다.\n', ...
                     getSafe(c, 'Type', 'Unknown'), getSafe(c, 'Name', 'Unnamed'));
        end
        % --- rhoL 결정 로직 끝 ---

        % 천(Canopy) 질량 계산: 표면 밀도 * 면적
        canopyArea = pi * (dia/2)^2;
        mCanopy = rho * canopyArea;

        % 줄 질량 계산 (이제 rhoL 사용 가능)
        nLine = getSafe(c.Geometry.ParachuteDetails, 'LineCount', 0);
        LenL = getSafe(c.Geometry.ParachuteDetails, 'LineLength', 0);
        mLine = nLine * LenL * rhoL;

        m = mCanopy + mLine; % 총 질량

        % CG 및 MOI 계산 (변경 없음)
        Lp = getSafe(c.Geometry, 'PackedLength', 0);
        Dp = getSafe(c.Geometry, 'PackedDiameter', 0);
        cg = [c.Position.AbsoluteStartX + Lp/2, 0, 0];

        if m > 1e-9 && Lp > 0 && Dp > 0 % 질량 0 아닐 때만 MOI 계산
             Ixx_cyl = 0.5 * m * (Dp/2)^2;
             Iyy_cyl = m * ((Dp/2)^2 / 4 + Lp^2 / 12);
             I = diag([Ixx_cyl, Iyy_cyl, Iyy_cyl]);
        else
             I = zeros(3);
        end
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

    compList_static(end+1) = struct('name', getSafe(c, 'Name', ['Component_' num2str(k)]), ...
                            'type', getSafe(c, 'Type', 'unknown'), ...
                            'mass', m, 'cg_ref', cg, 'inertia_cg', I);
end

fprintf('정적 부품 질량 특성 계산 완료: 총 %d개 부품 처리됨\n', numel(compList_static));

% 총 질량 및 무게중심 계산
totalMass = 0;
weightedCG = [0, 0, 0];

for i = 1:numel(compList_static)
    totalMass = totalMass + compList_static(i).mass;
    weightedCG = weightedCG + compList_static(i).mass * compList_static(i).cg_ref;
end

if totalMass > 0
    weightedCG = weightedCG / totalMass;
end
% 
% % 질량 특성 요약 출력
% fprintf('\n====== 정적 부품 질량 특성 요약 ======\n');
% fprintf('총 질량: %.3f kg\n', totalMass);
% fprintf('무게중심 위치: [%.3f, %.3f, %.3f] m\n', weightedCG);
% 
% % 부품별 질량 및 무게중심 출력 (표 형식)
% fprintf('\n====== 개별 부품 질량 특성 ======\n');
% fprintf('%-30s %-15s %-10s %-30s\n', '부품명', '유형', '질량(kg)', '무게중심 [x, y, z](m)');
% fprintf('%-30s %-15s %-10s %-30s\n', repmat('-', 1, 30), repmat('-', 1, 15), repmat('-', 1, 10), repmat('-', 1, 30));

for i = 1:numel(compList_static)
    fprintf('%-30s %-15s %-10.3f [%-8.3f, %-8.3f, %-8.3f]\n', ...
        truncateString(compList_static(i).name, 30), ...
        compList_static(i).type, ...
        compList_static(i).mass, ...
        compList_static(i).cg_ref(1), compList_static(i).cg_ref(2), compList_static(i).cg_ref(3));
end

% % 질량 기준 정렬된 부품 목록 출력
% [~, massOrder] = sort([compList_static.mass], 'descend');
% fprintf('\n====== 질량 기준 상위 컴포넌트 ======\n');
% for i = 1:min(10, numel(compList_static))
%     idx = massOrder(i);
%     fprintf('%s: %.3f kg\n', compList_static(idx).name, compList_static(idx).mass);
% end

% fprintf('\n====== 처리 완료 ======\n\n');

end

% === 유틸리티 함수들 ===

function warnSkip(c, reason)
fprintf('경고: %s (%s) - %s. 건너뜀.\n', getSafe(c, 'Type', 'Unknown'), getSafe(c, 'Name', 'Unnamed'), reason);
end

function [m, cg, Icg] = calcCylMOI(Ro, Ri, L, rho)
    V = pi*(Ro^2 - Ri^2)*L; 
    m = rho*V;
    cg = [L/2 0 0];
    Ixx = 0.5*m*(Ro^2 + Ri^2);
    Iyy = m*((Ro^2 + Ri^2)/4 + L^2/12);
    Icg = diag([Ixx, Iyy, Iyy]);
end

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

function [m, cg, I] = calcPlateMOI(verts, t, rho)
% 평판 핀의 질량, 무게중심, 관성 모멘트 계산
    [A, cent, Iz] = polygeom(verts(:,1), verts(:,2));
    m = 0.5 * A * t * rho; 
    cg = cent;  % 2D 좌표
    Ixx = m * (cent(2)^2 + t^2/12);
    Iyy = m * (cent(1)^2 + t^2/12);
    Izz = Iz * t * rho;
    I = diag([Ixx, Iyy, Izz]);
end

function [A, cent, Izz] = polygeom(x, y)
    % 다각형의 면적, 무게중심, 2차 모멘트 계산
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

function v = getSafe(S, field, def)
% getSafe  구조체에서 안전하게 필드 값을 추출하는 함수
%
% v = getSafe(S, field, def)
%
% 입력:
% S: 대상 구조체
% field: 추출할 필드 이름 ('a.b.c' 형식의 경로 또는 {'A', 'B'} 형식의 대안 목록)
% def: 필드가 없거나 값이 비어있을 경우 반환할 기본값
%
% 출력:
% v: 추출된 값 또는 기본값

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

function s = truncateString(str, maxLen)
% 문자열을 지정된 최대 길이로 자르고 필요시 ... 추가
if length(str) > maxLen
    s = [str(1:maxLen-3) '...'];
else
    s = str;
end
end