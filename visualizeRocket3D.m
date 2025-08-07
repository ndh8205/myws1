function figHandle = visualizeRocket3D(compList, flatRocketParams)
    % visualizeRocket3D  로켓 부품의 3D 모델과 질량중심 위치를 시각화
    %
    % figHandle = visualizeRocket3D(compList, flatRocketParams)
    %
    % 입력:
    % compList : bclist 함수에서 계산된 부품별 질량특성 정보 구조체 배열
    % flatRocketParams : flattenRocketStructure로 얻은 평탄화된 로켓 파라미터 구조체
    %
    % 출력:
    % figHandle : 생성된 그림 핸들
    
    fprintf('\n====== 로켓 3D 시각화 ======\n');
    
    % 새 Figure 생성
    figHandle = figure('Name', '로켓 3D 모델과 질량 특성', 'Color', 'w', 'Position', [100, 100, 1200, 600]);
    
    % 두 개의 서브플롯 설정
    subplot(1, 2, 1);
    ax1 = gca;
    hold(ax1, 'on');
    grid(ax1, 'on');
    axis(ax1, 'equal');
    view(ax1, 3);
    title(ax1, '로켓 3D 모델');
    xlabel(ax1, 'X (m)');
    ylabel(ax1, 'Y (m)');
    zlabel(ax1, 'Z (m)');
    
    subplot(1, 2, 2);
    ax2 = gca;
    hold(ax2, 'on');
    grid(ax2, 'on');
    axis(ax2, 'equal');
    view(ax2, 3);
    title(ax2, '부품 CG 위치');
    xlabel(ax2, 'X (m)');
    ylabel(ax2, 'Y (m)');
    zlabel(ax2, 'Z (m)');
    
    % 부품 유형별 색상 정의 - 모두 RGB 벡터 형태로 지정
    colorMap = struct(...
        'bodytube', [0.3, 0.3, 0.3], ...      % 회색
        'tubecoupler', [0.5, 0.5, 0.5], ...   % 연한 회색
        'nosecone', [0, 0.3, 0.8], ...        % 파랑 (더 진한색)
        'transition', [0.8, 0.2, 0.8], ...    % 마젠타
        'fin', [0.8, 0.2, 0.2], ...           % 빨강 (더 진한색)
        'centeringring', [0.8, 0.6, 0.2], ... % 주황-갈색
        'bulkhead', [0.6, 0.4, 0.2], ...      % 갈색
        'parachute', [0.2, 0.6, 0.8], ...     % 하늘색
        'masscomponent', [0.7, 0.7, 0.7], ... % 은색
        'unknown', [0.5, 0.5, 0.5]);          % 회색
    
    % flatRocketParams를 사용하여 3D 모델 그리기
    fprintf('3D 로켓 모델 그리기 (%d개 부품)...\n', numel(flatRocketParams));
    [rocket_max_length, rocket_max_radius] = drawRocketModel(ax1, flatRocketParams, colorMap);
    
    % 부품 CG 점 플롯
    fprintf('%d개 부품의 CG 점 플롯...\n', numel(compList));
    for i = 1:numel(compList)
        comp = compList(i);
        
        % 해당 부품 유형의 색상 가져오기
        if isfield(colorMap, lower(comp.type))
            color = colorMap.(lower(comp.type));
        else
            color = colorMap.unknown;
        end
        
        % 부품 질량에 비례하는 마커 크기 계산
        markerSize = max(20, min(200, 30 * sqrt(comp.mass) + 10));
        
        % CG 점 플롯
        scatter3(ax2, comp.cg_ref(1), comp.cg_ref(2), comp.cg_ref(3), markerSize, color, 'filled');
        
        % 부품 이름 텍스트 라벨 추가
        text(ax2, comp.cg_ref(1), comp.cg_ref(2), comp.cg_ref(3), [' ' comp.name], 'FontSize', 8);
        
        % 로켓 크기 업데이트
        rocket_max_length = max(rocket_max_length, comp.cg_ref(1) + 0.5);
        rocket_max_radius = max(rocket_max_radius, sqrt(comp.cg_ref(2)^2 + comp.cg_ref(3)^2) + 0.5);
    end
    
    % 전체 CG 계산 및 플롯
    totalMass = sum([compList.mass]);
    
    if totalMass > 0
        cgTotal = [0, 0, 0];
        for i = 1:numel(compList)
            cgTotal = cgTotal + compList(i).mass * compList(i).cg_ref;
        end
        cgTotal = cgTotal / totalMass;
        
        % CG 시각화에 전체 CG 플롯
        scatter3(ax2, cgTotal(1), cgTotal(2), cgTotal(3), 200, 'k', 'filled', 'MarkerEdgeColor', 'w');
        text(ax2, cgTotal(1), cgTotal(2), cgTotal(3), ' 전체 CG', 'FontWeight', 'bold', 'FontSize', 12);
        
        % 3D 모델에도 전체 CG 플롯
        scatter3(ax1, cgTotal(1), cgTotal(2), cgTotal(3), 200, 'k', 'filled', 'MarkerEdgeColor', 'w');
        text(ax1, cgTotal(1), cgTotal(2), cgTotal(3), ' 전체 CG', 'FontWeight', 'bold', 'FontSize', 12);
    end
    
    % CG 시각화를 위한 적절한 축 범위 설정
    padding = max(rocket_max_radius, rocket_max_length * 0.01) * 0.03;
    xlim(ax2, [0 - padding, rocket_max_length + padding]);
    ylim(ax2, [-rocket_max_radius - padding, rocket_max_radius + padding]);
    zlim(ax2, [-rocket_max_radius - padding, rocket_max_radius + padding]);
    
    % CG 시각화에 범례 추가
    uniqueTypes = unique({compList.type});
    legendHandles = [];
    legendLabels = {};
    
    for i = 1:numel(uniqueTypes)
        type = uniqueTypes{i};
        if isfield(colorMap, lower(type))
            color = colorMap.(lower(type));
        else
            color = colorMap.unknown;
        end
        h = scatter3(ax2, NaN, NaN, NaN, 50, color, 'filled');
        legendHandles = [legendHandles, h];
        legendLabels{end+1} = type;
    end
    
    h = scatter3(ax2, NaN, NaN, NaN, 100, 'k', 'filled', 'MarkerEdgeColor', 'w');
    legendHandles = [legendHandles, h];
    legendLabels{end+1} = '전체 CG';
    
    legend(ax2, legendHandles, legendLabels, 'Location', 'best');
    
    % 더 나은 3D 외관을 위한 조명 추가
    lighting(ax1, 'phong');
    camlight(ax1, 'left');
    material(ax1, 'dull');
    
    lighting(ax2, 'phong');
    camlight(ax2, 'left');
    material(ax2, 'dull');
    
    fprintf('3D 시각화 완료\n\n');

    % 로켓 3D 모델만 보여주는 별도 figure 생성
    figRocket = figure('Name', '로켓 3D 모델', 'Color', 'w', 'Position', [500, 100, 900, 800]);
    axRocket = gca;
    hold(axRocket, 'on');
    grid(axRocket, 'on');
    
    % 축 비율을 직사각형으로 변경 (더 가까이 보이게)
    axis(axRocket, 'equal');
    
    % 제목과 라벨 설정
    title(axRocket, '로켓 3D 모델');
    xlabel(axRocket, 'X (m)');
    ylabel(axRocket, 'Y (m)');
    zlabel(axRocket, 'Z (m)');
    
    % 로켓 모델 그리기 (기존 colorMap 재사용)
    [rocket_length] = drawRocketModel(axRocket, flatRocketParams, colorMap);
    
    % 로켓에 맞게 축 범위 매우 좁게 설정 (가까이 보이게)
    length_padding = rocket_length * 0.3; % 매우 작은 패딩
    
    xlim(axRocket, [0 - length_padding, rocket_length + length_padding]);
    ylim(axRocket, [-0.3, 0.3]);
    zlim(axRocket, [-0.3, 0.3]);
    
    % 더 가까이 보이는 각도로 설정
    view(axRocket, [20, 15]);
    
    % 조명 추가
    lighting(axRocket, 'phong');
    camlight(axRocket, 'left');
    material(axRocket, 'dull');
    
    % 추가 확대 (로켓에 더 집중)
    zoom(axRocket, 1.3);
    
    fprintf('별도 3D 모델 시각화 완료\n\n');
    
    % 원래 figure로 포커스 되돌리기
    figure(figHandle);
    
end

function [rocket_max_length, rocket_max_radius] = drawRocketModel(ax, flatRocketParams, colorMap)
    % flatRocketParams를 기반으로 로켓의 3D 모델 그리기
    
    n_theta = 40; % 원주 방향 점 개수 증가 (더 부드러운 표면)
    
    % 로켓 크기 추적 변수 초기화
    rocket_max_length = 0;
    rocket_max_radius = 0;
    
    % 핀셋 인덱스 추적
    finSetIndices = [];
    
    % 먼저 non-fin 부품들 모두 그리기
    for i = 1:numel(flatRocketParams)
        currentComponent = flatRocketParams(i);
        
        % 유효하지 않은 부품 건너뛰기
        if ~isstruct(currentComponent) || ~isfield(currentComponent, 'Type') || ~isfield(currentComponent, 'Name') || ...
           ~isfield(currentComponent,'Position') || ~isfield(currentComponent.Position,'AbsoluteStartX') || ...
           ~isfield(currentComponent, 'Geometry')
            continue;
        end
        
        % 핀셋은 나중에 따로 처리
        if strcmpi(currentComponent.Type, 'trapezoidfinset')
            finSetIndices(end+1) = i;
            continue;
        end
        
        compName = currentComponent.Name;
        compType = currentComponent.Type;
        startX = currentComponent.Position.AbsoluteStartX;
        
        % 유효하지 않은 위치 건너뛰기
        if isnan(startX)
            continue;
        end
        
        % 해당 부품 유형의 색상 가져오기
        if isfield(colorMap, lower(compType))
            color = colorMap.(lower(compType));
        else
            color = colorMap.unknown;
        end
        
        % 부품 타입별 그리기
        try
            switch lower(compType)
                case 'nosecone'
                    geo = currentComponent.Geometry;
                    % 필수 지오메트리 필드 확인
                    if ~isfield(geo, 'Length') || ~isfield(geo, 'AftRadius') || ~isfield(geo, 'Shape') || ...
                       isnan(geo.Length) || isnan(geo.AftRadius)
                        continue;
                    end
                    
                    len = geo.Length;
                    aftRadius = geo.AftRadius;
                    shapeType = geo.Shape;
                    shapeParam = NaN; 
                    if isfield(geo, 'ShapeParameter'), shapeParam = geo.ShapeParameter; end
                    if isnan(shapeParam), shapeParam = 0; end
                    
                    drawNoseCone3D(ax, shapeType, shapeParam, len, aftRadius, startX, color, n_theta);
                    
                    rocket_max_radius = max(rocket_max_radius, aftRadius);
                    rocket_max_length = max(rocket_max_length, startX + len);
                    
                    % 노즈콘 어깨 부분 그리기
                    if isfield(geo, 'AftShoulder') && isfield(geo.AftShoulder, 'Length') && isfield(geo.AftShoulder, 'Radius') && ...
                       ~isnan(geo.AftShoulder.Length) && geo.AftShoulder.Length > 0 && ...
                       ~isnan(geo.AftShoulder.Radius) && geo.AftShoulder.Radius > 0
                        shoulderStartX = startX + len;
                        shoulderLen = geo.AftShoulder.Length;
                        shoulderRadius = geo.AftShoulder.Radius;
                        shoulderThickness = NaN;
                        if isfield(geo.AftShoulder, 'Thickness'), shoulderThickness = geo.AftShoulder.Thickness; end
                        
                        drawBodyTube3D(ax, shoulderStartX, shoulderLen, shoulderRadius, color, n_theta);
                        
                        rocket_max_radius = max(rocket_max_radius, shoulderRadius);
                        rocket_max_length = max(rocket_max_length, shoulderStartX + shoulderLen);
                    end
                    
                case 'bodytube'
                    geo = currentComponent.Geometry;
                    % 필수 지오메트리 필드 확인
                    if ~isfield(geo, 'Length') || isnan(geo.Length)
                        continue;
                    end
                    
                    len = geo.Length;
                    thickness = NaN;
                    if isfield(geo, 'Thickness'), thickness = geo.Thickness; end
                    
                    % Radius 필드는 다양한 이름으로 존재할 수 있음
                    radius = NaN;
                    if isfield(geo, 'Radius') && ~isnan(geo.Radius)
                        radius = geo.Radius;
                    elseif isfield(geo, 'OuterRadius') && ~isnan(geo.OuterRadius)
                        radius = geo.OuterRadius;
                    end
                    
                    if isnan(radius)
                        % Position에서 EndRadius 확인 (부모 반경)
                        if isfield(currentComponent.Position, 'EndRadius') && ~isnan(currentComponent.Position.EndRadius)
                            radius = currentComponent.Position.EndRadius;
                        else
                            % 기본값 설정
                            radius = 0.05; % 5cm 기본값
                        end
                    end
                    
                    drawBodyTube3D(ax, startX, len, radius, color, n_theta);
                    
                    rocket_max_radius = max(rocket_max_radius, radius);
                    rocket_max_length = max(rocket_max_length, startX + len);
                    
                    % 모터 마운트가 있는 경우 모터 그리기
                    if isfield(currentComponent, 'MotorMount') && isfield(currentComponent.MotorMount, 'HasMount') && ...
                       currentComponent.MotorMount.HasMount && isfield(currentComponent.MotorMount, 'Motor')
                        motor = currentComponent.MotorMount.Motor;
                        if isfield(motor, 'Diameter') && isfield(motor, 'Length') && ...
                           ~isnan(motor.Diameter) && ~isnan(motor.Length)
                            motorDiameter = motor.Diameter;
                            motorLength = motor.Length;
                            motorOverhang = 0;
                            if isfield(currentComponent.MotorMount, 'MotorOverhang') && ~isnan(currentComponent.MotorMount.MotorOverhang)
                                motorOverhang = currentComponent.MotorMount.MotorOverhang;
                            end
                            
                            % 모터 위치 계산 (BottomOfTube - MotorLength + Overhang)
                            motorStartX = startX + len - motorLength + motorOverhang;
                            motorRadius = motorDiameter / 2;
                            
                            % 모터 색상 (회색)
                            motorColor = [0.4, 0.4, 0.4];
                            
                            drawBodyTube3D(ax, motorStartX, motorLength, motorRadius, motorColor, n_theta);
                            
                            % 노즐 표시 (빨간색)
                            nozzleLength = min(0.05, motorLength * 0.1);
                            nozzleRadius = motorRadius * 0.8;
                            nozzleColor = [0.8, 0.2, 0.2];
                            
                            drawBodyTube3D(ax, motorStartX + motorLength - nozzleLength, nozzleLength, nozzleRadius, nozzleColor, n_theta);
                        end
                    end
                    
                case 'transition'
                    geo = currentComponent.Geometry;
                    % 필수 지오메트리 필드 확인
                    if ~isfield(geo, 'Length') || ~isfield(geo, 'ForeRadius') || ~isfield(geo, 'AftRadius') || ...
                       isnan(geo.Length) || isnan(geo.ForeRadius) || isnan(geo.AftRadius)
                        continue;
                    end
                    
                    len = geo.Length;
                    foreRadius = geo.ForeRadius;
                    aftRadius = geo.AftRadius;
                    shape = 'conical'; % 기본값
                    if isfield(geo, 'Shape'), shape = geo.Shape; end
                    
                    drawTransition3D(ax, startX, len, foreRadius, aftRadius, shape, color, n_theta);
                    
                    rocket_max_radius = max(rocket_max_radius, max(foreRadius, aftRadius));
                    rocket_max_length = max(rocket_max_length, startX + len);
                    
                    % 전방 숄더 그리기
                    if isfield(geo, 'ForeShoulder') && isfield(geo.ForeShoulder, 'Length') && isfield(geo.ForeShoulder, 'Radius') && ...
                       ~isnan(geo.ForeShoulder.Length) && geo.ForeShoulder.Length > 0 && ...
                       ~isnan(geo.ForeShoulder.Radius) && geo.ForeShoulder.Radius > 0
                        shoulderStartX = startX - geo.ForeShoulder.Length;
                        shoulderLen = geo.ForeShoulder.Length;
                        shoulderRadius = geo.ForeShoulder.Radius;
                        
                        drawBodyTube3D(ax, shoulderStartX, shoulderLen, shoulderRadius, color, n_theta);
                        
                        rocket_max_radius = max(rocket_max_radius, shoulderRadius);
                        rocket_max_length = max(rocket_max_length, startX); % 앞쪽으로 확장
                    end
                    
                    % 후방 숄더 그리기
                    if isfield(geo, 'AftShoulder') && isfield(geo.AftShoulder, 'Length') && isfield(geo.AftShoulder, 'Radius') && ...
                       ~isnan(geo.AftShoulder.Length) && geo.AftShoulder.Length > 0 && ...
                       ~isnan(geo.AftShoulder.Radius) && geo.AftShoulder.Radius > 0
                        shoulderStartX = startX + len;
                        shoulderLen = geo.AftShoulder.Length;
                        shoulderRadius = geo.AftShoulder.Radius;
                        
                        drawBodyTube3D(ax, shoulderStartX, shoulderLen, shoulderRadius, color, n_theta);
                        
                        rocket_max_radius = max(rocket_max_radius, shoulderRadius);
                        rocket_max_length = max(rocket_max_length, shoulderStartX + shoulderLen);
                    end
                    
                case {'centeringring', 'bulkhead'}
                    geo = currentComponent.Geometry;
                    % 필수 지오메트리 필드 확인 - 다양한 필드명 지원
                    outerRadius = NaN;
                    if isfield(geo, 'OuterRadius') && ~isnan(geo.OuterRadius)
                        outerRadius = geo.OuterRadius;
                    elseif isfield(geo, 'Radius') && ~isnan(geo.Radius)
                        outerRadius = geo.Radius;
                    end
                    
                    thickness = NaN;
                    if isfield(geo, 'Thickness') && ~isnan(geo.Thickness)
                        thickness = geo.Thickness;
                    elseif isfield(geo, 'Length') && ~isnan(geo.Length)
                        thickness = geo.Length; % 일부 XML에서는 Length가 실제 두께
                    end
                    
                    innerRadius = 0; % 기본값 (bulkhead는 내부가 없음)
                    if strcmpi(compType, 'centeringring') && isfield(geo, 'InnerRadius') && ~isnan(geo.InnerRadius)
                        innerRadius = geo.InnerRadius;
                    end
                    
                    if isnan(outerRadius) || outerRadius <= 0 || isnan(thickness) || thickness <= 0
                        continue;
                    end
                    
                    drawRing3D(ax, startX, thickness, outerRadius, innerRadius, color, n_theta);
                    
                    rocket_max_radius = max(rocket_max_radius, outerRadius);
                    rocket_max_length = max(rocket_max_length, startX + thickness);
                    
                case {'masscomponent', 'parachute'}
                    % 간단한 원통형 또는 박스로 표현 (질량 부품, 낙하산 등)
                    geo = currentComponent.Geometry;
                    packedLength = NaN;
                    packedRadius = NaN;
                    
                    if isfield(geo, 'PackedLength') && ~isnan(geo.PackedLength)
                        packedLength = geo.PackedLength;
                    end
                    
                    if isfield(geo, 'PackedRadius') && ~isnan(geo.PackedRadius)
                        packedRadius = geo.PackedRadius;
                    end
                    
                    if isnan(packedLength) || packedLength <= 0
                        packedLength = 0.05; % 기본값
                    end
                    
                    if isnan(packedRadius) || packedRadius <= 0
                        % 부모 반경의 50%로 설정
                        if isfield(currentComponent.Position, 'EndRadius') && ~isnan(currentComponent.Position.EndRadius)
                            packedRadius = currentComponent.Position.EndRadius * 0.5;
                        else
                            packedRadius = 0.02; % 기본값
                        end
                    end
                    
                    % Parachute의 경우 추가 표시
                    if strcmpi(compType, 'parachute') && isfield(geo, 'Diameter') && ~isnan(geo.Diameter)
                        % 예상 직경 표시 (투명한 구체)
                        parachuteDiameter = geo.Diameter;
                        [x, y, z] = sphere(20);
                        x = x * parachuteDiameter/2;
                        y = y * parachuteDiameter/2;
                        z = z * parachuteDiameter/2;
                        
                        % 파라슈트 중심은 packed 위치로부터 조금 위쪽
                        paraCenter = startX + packedLength/2;
                        
                        % 반투명한 구체로 표시
                        % surf(ax, x + paraCenter, y, z, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 0.1);
                        
                        % 반경과 길이 업데이트
                        rocket_max_radius = max(rocket_max_radius, parachuteDiameter/2);
                    end
                    
                    % 실제 packed 형태 그리기
                    drawSimpleComponent3D(ax, startX, packedLength, packedRadius, color, n_theta);
                    
                    rocket_max_length = max(rocket_max_length, startX + packedLength);
                    rocket_max_radius = max(rocket_max_radius, packedRadius);
                    
                otherwise
                    % 지원되지 않는 부품 타입은 기본적인 형태로 그리기
                    fprintf('  정보: 부품 타입 "%s"에 대한 상세 렌더링 로직이 없습니다. 기본 원통형으로 표시합니다.\n', compType);
                    
                    len = 0.05; % 기본 길이
                    if isfield(currentComponent.Geometry, 'Length') && ~isnan(currentComponent.Geometry.Length)
                        len = currentComponent.Geometry.Length;
                    elseif isfield(currentComponent.Position, 'Length') && ~isnan(currentComponent.Position.Length)
                        len = currentComponent.Position.Length;
                    end
                    
                    radius = 0.05; % 기본 반경
                    if isfield(currentComponent.Geometry, 'Radius') && ~isnan(currentComponent.Geometry.Radius)
                        radius = currentComponent.Geometry.Radius;
                    elseif isfield(currentComponent.Position, 'EndRadius') && ~isnan(currentComponent.Position.EndRadius)
                        radius = currentComponent.Position.EndRadius;
                    end
                    
                    drawSimpleComponent3D(ax, startX, len, radius, color, n_theta);
                    
                    rocket_max_radius = max(rocket_max_radius, radius);
                    rocket_max_length = max(rocket_max_length, startX + len);
            end
        catch e
            fprintf('  경고: 부품 "%s"(%s) 그리기 중 오류 발생: %s\n', compName, compType, e.message);
            continue;
        end
    end
    
    % 핀셋 그리기 (visualizeFinSet의 핵심 부분 적용)
    if ~isempty(finSetIndices)
        fprintf('핀셋 %d개 발견, 3D 모델에 그리는 중...\n', numel(finSetIndices));
        
        for i = 1:numel(finSetIndices)
            fs = flatRocketParams(finSetIndices(i));
            
            % 안전-필드 읽기 함수 (visualizeFinSet에서 가져옴)
            geo = safeStruct(fs, 'Geometry');
            inst = safeStruct(fs, 'InstanceInfo');
            pos = safeStruct(fs, 'Position');
            
            g = @(k,d) getField(geo, k, d);
            i_ = @(k,d) getField(inst, k, d);
            p = @(k,d) getField(pos, k, d);
            
            % 기본 핀 치수
            rootChord = g({'RootChord','rootchord'}, NaN);
            tipChord  = g({'TipChord','tipchord'},  0);
            finHeight = g({'Height','height'}, NaN);
            sweepLen  = g({'Sweep','sweeplength'}, 0);
            thickness = g({'Thickness','thickness'}, 0.003);
            
            if any(isnan([rootChord, finHeight]))
                fprintf('  경고: 핀 "%s"의 필수 치수 (RootChord/Height) 누락됨\n', fs.Name);
                continue;
            end
            
            % 핀셋 배치 정보
            Nf = i_({'Count','count'}, 1);
            radPos = i_({'RadialPosition','radialposition'}, 0);
            radDir = i_({'RadialDirection','radialdirection'}, 0);
            angOff = i_({'AngleOffset','angleoffset'}, 0);
            
            % 위치 정보
            xRear = p({'AbsoluteStartX','x'}, 0);          % 루트 뒤
            attachR = p({'EndRadius','endradius'}, 0) + radPos;
            if attachR <= 0, attachR = 0.05; end
            
            % 캔트 각도
            cantAngle = deg2rad(getField(fs, 'CantAngle', 0));
            
            % 핀 색상
            if isfield(colorMap, 'fin')
                finClr = colorMap.fin;
            else
                finClr = [0.8, 0.2, 0.2]; % 기본 빨간색
            end
            
            % 핀 꼭지점(2D) 계산 - 꼬임 없는 순서 (root 뒤→root 앞→tip 앞→tip 뒤)
            xFront = xRear - rootChord;
            xTipA  = xFront + sweepLen;           % tip 앞
            xTipB  = xTipA  + tipChord;           % tip 뒤
            verts = [ xRear, attachR;             % 1 root 뒤
                      xFront, attachR;            % 2 root 앞
                      xTipA, attachR+finHeight;   % 3 tip 앞
                      xTipB, attachR+finHeight];  % 4 tip 뒤
            
            % 회전 각도 배열
            baseDeg = (0:Nf-1)*(360/Nf) + radDir + angOff;
            
            % 단면 형태 및 필렛 정보
            crossSection = getField(fs, 'CrossSection', 'SQUARE'); % 단면 형태
            filletRadius = getField(fs, 'FilletRadius', 0);        % 필렛 반경
            
            % 단면 형태에 따른 두께 조정 (시각적 표현)
            if strcmpi(crossSection, 'AIRFOIL')
                effectiveThickness = thickness * 0.85; % 에어포일은 얇게 보임
            elseif strcmpi(crossSection, 'ROUNDED')
                effectiveThickness = thickness * 0.99; % 둥근 단면은 거의 원래 두께
            else
                effectiveThickness = thickness;        % 기본 사각 단면
            end
            
            % 3D 모델에 핀 그리기
            for k = 1:Nf
                phi = deg2rad(baseDeg(k));
                
                % 캔트 각도 적용 (X-Z 평면 회전)
                Rcant = eye(3);
                if abs(cantAngle) > 1e-6
                    Rcant = [
                        cos(cantAngle), 0, sin(cantAngle);
                        0, 1, 0;
                        -sin(cantAngle), 0, cos(cantAngle)
                    ];
                end
                
                % 기본 회전 (Y-Z 평면)
                v3 = zeros(size(verts, 1), 3);
                v3(:,1) = verts(:,1);
                v3(:,2) = verts(:,2).*cos(phi);
                v3(:,3) = -verts(:,2).*sin(phi);
                
                % 캔트 적용 (필요시)
                if abs(cantAngle) > 1e-6
                    for j = 1:size(v3, 1)
                        rotated = (Rcant * v3(j, :)')';
                        v3(j, :) = rotated;
                    end
                end
                
                % 두께 적용
                n = cross(v3(2,:)-v3(1,:), v3(3,:)-v3(2,:)); 
                n = n/norm(n)*effectiveThickness/2;
                vf = v3-n; 
                vb = v3+n; 
                verts3D = [vf;vb];
                side = [1 2 6 5; 3 4 8 7; 2 3 7 6; 1 4 8 5];
                
                % 핀 면 그리기
                patch('Parent', ax, 'Vertices', vf, 'Faces', [1 2 3 4], 'FaceColor', finClr, 'EdgeColor', 'k', 'FaceAlpha', 0.9);
                patch('Parent', ax, 'Vertices', vb, 'Faces', [1 2 3 4], 'FaceColor', finClr*0.8, 'EdgeColor', 'k', 'FaceAlpha', 0.9);
                patch('Parent', ax, 'Vertices', verts3D, 'Faces', side, 'FaceColor', finClr*0.9, 'EdgeColor', 'k', 'FaceAlpha', 0.9);
                
                % 필렛 그리기 (있는 경우)
                if filletRadius > 0
                    [xf, yf, zf] = cylinder(linspace(0, filletRadius, 10), 20);
                    xf = xf * 0.5;  % 반원 형태로 조정
                    
                    % 필렛 위치 및 방향 조정
                    xf = xf * rootChord + xFront;
                    yf = yf + attachR;
                    
                    % 회전 적용
                    yfRot = yf .* cos(phi);
                    zfRot = -yf .* sin(phi);
                    
                    % 캔트 적용 (필요시)
                    if abs(cantAngle) > 1e-6
                        for j = 1:numel(xf)
                            pt = [xf(j), yfRot(j), zfRot(j)];
                            rotated = (Rcant * pt')';
                            xf(j) = rotated(1);
                            yfRot(j) = rotated(2);
                            zfRot(j) = rotated(3);
                        end
                    end
                    
                    % 필렛 표시
                    surf(ax, xf, yfRot, zfRot, 'FaceColor', finClr*0.7, 'EdgeColor', 'none', 'FaceAlpha', 0.7);
                end
            end
            
            % 로켓 최대 반경 업데이트
            rocket_max_radius = max(rocket_max_radius, attachR + finHeight);
            rocket_max_length = max(rocket_max_length, xRear);
        end
    end
    
    % 적절한 축 범위 설정
    padding = max(rocket_max_radius, rocket_max_length * 0.1) * 0.3;
    xlim(ax, [0 - padding, rocket_max_length + padding]);
    ylim(ax, [-rocket_max_radius - padding, rocket_max_radius + padding]);
    zlim(ax, [-rocket_max_radius - padding, rocket_max_radius + padding]);
    
    % 원점에 축 그리기
    origin = [0, 0, 0];
    arrow_length_x = rocket_max_length * 0.05;
    arrow_length_yz = max(rocket_max_radius * 0.5, 0.05);
    max_head_size = 0.1;
    
    quiver3(ax, origin(1), origin(2), origin(3), arrow_length_x, 0, 0, 0, 'r', 'LineWidth', 1.5, 'MaxHeadSize', max_head_size);
    quiver3(ax, origin(1), origin(2), origin(3), 0, arrow_length_yz, 0, 0, 'g', 'LineWidth', 1.5, 'MaxHeadSize', max_head_size);
    quiver3(ax, origin(1), origin(2), origin(3), 0, 0, arrow_length_yz, 0, 'b', 'LineWidth', 1.5, 'MaxHeadSize', max_head_size);
end

% 헬퍼 함수들

% bclist에서 가져온 헬퍼 함수들
function s = safeStruct(p, f)
    if isfield(p, f) && isstruct(p.(f))
        s = p.(f); 
    else
        s = struct(); 
    end
end

function v = getField(S, keys, def)
    v = def; 
    if ~isstruct(S), return, end; 
    if ~iscell(keys), keys = {keys}; end
    
    for k = 1:numel(keys)
        if isfield(S, keys{k})
            tmp = S.(keys{k});
            if ~isempty(tmp) && ~(isnumeric(tmp) && isnan(tmp))
                v = tmp; 
                return; 
            end
        end
    end
end

% 컴포넌트 그리기 함수들
function drawNoseCone3D(ax, shapeType, shapeParam, L, R, start_x, color, n_theta)
    % 노즈콘 3D 그리기
    n_points = 50; % 형상 곡선을 그릴 점 개수
    x_local = linspace(0, L, n_points); % 노즈콘 로컬 X 좌표 (0부터 L까지)
    y_local = zeros(1, n_points);     % 해당 X에서의 반경 (로컬 Y)
    
    % 기본 유효성 검사: 길이와 끝 반경은 0보다 커야 함
    if L <= 0 || R <= 0
        return;
    end
    
    % 형상 타입에 따른 반경 프로파일 계산
    try
        switch lower(shapeType)
            case 'ogive' % 원호(Ogive) 형태
                rho = (R^2 + L^2) / (2*R); % 원호의 반경
                y_local = sqrt(max(0, rho^2 - (L - x_local).^2)) + R - rho;
            case 'conical' % 원뿔(Conical) 형태
                y_local = R * (x_local / L);
            case 'elliptical' % 타원(Elliptical) 형태 (반 타원)
                y_local = R * sqrt(max(0, 1 - ((x_local-L)/L).^2));
            case 'power' % Power-Series 형태 (n=shapeParam)
                n_pow = shapeParam; % shapeParam은 Power 지수
                % 유효하지 않은 Power 지수는 기본값 0.5 사용
                if isnan(n_pow) || n_pow <= 0 || n_pow > 1
                    n_pow = 0.5;
                end
                y_local = R * (x_local / L).^n_pow;
            case 'haack' % Haack Series 형태 (k=shapeParam)
                k = shapeParam; % shapeParam은 Haack 상수 k
                % 유효하지 않은 Haack 상수는 기본값 0 사용
                if isnan(k) || abs(k) > 1
                    k = 0; % LV-Haack
                end
                
                % Haack 공식
                theta = acos(max(-1, min(1, 1 - 2*x_local/L)));
                if k == 0 % LV-Haack
                    y_local = (R / sqrt(pi)) * sqrt(max(0, theta - 0.5*sin(2*theta)));
                else % 일반 Haack
                    y_local = (R / sqrt(pi)) * sqrt(max(0, theta - 0.5*sin(2*theta) + k*sin(theta).^3));
                end
            otherwise % 지원되지 않는 형태는 원뿔형으로 근사
                y_local = R * (x_local / L);
        end
        
        % 계산된 로컬 Y 값 정리
        y_local(isnan(y_local) | ~isreal(y_local)) = 0; % NaN 또는 복소수 결과를 0으로 처리
        y_local = max(0, y_local); % 음수 반경을 0으로 처리
        y_local(1) = 0; % 시작점 반경은 0
        y_local(end) = R; % 끝점 반경은 R
        
    catch
        % 오류 발생 시 기본 원뿔형으로 처리
        y_local = R * (x_local / L);
    end
    
    % 3D 표면 생성
    theta = linspace(0, 2*pi, n_theta);
    
    % 표면 좌표 계산
    X_surf = start_x + repmat(x_local(:), 1, n_theta);
    Y_surf = repmat(y_local(:), 1, n_theta) .* repmat(cos(theta), n_points, 1);
    Z_surf = repmat(y_local(:), 1, n_theta) .* repmat(sin(theta), n_points, 1);
    
    % 표면 플롯
    surf(ax, X_surf, Y_surf, Z_surf, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 0.3);
    
    % 앞면(노즈 팁) 그리기 - 원뿔/ogive 타입에는 필요 없음
    if ~strcmpi(shapeType, 'conical') && ~strcmpi(shapeType, 'ogive')
        % 원형 디스크 생성 (노즈 팁)
        r_tip = max(0, y_local(1));  % 노즈 팁 반경 (보통 0)
        if r_tip > 0
            [x_disk, y_disk] = meshgrid(linspace(-r_tip, r_tip, 10), linspace(-r_tip, r_tip, 10));
            valid_pts = x_disk.^2 + y_disk.^2 <= r_tip^2;
            x_disk = x_disk(valid_pts);
            y_disk = y_disk(valid_pts);
            z_disk = zeros(size(x_disk));
            
            % 노즈 팁 위치에 원판 그리기
            fill3(ax, start_x + zeros(size(x_disk)), y_disk, x_disk, color, 'EdgeColor', 'none');
        end
    end
end

function drawBodyTube3D(ax, start_x, len, radius, color, n_theta)
    % 바디튜브 3D 그리기
    if len <= 0 || radius <= 0
        return;
    end
    
    % 원통 생성
    theta = linspace(0, 2*pi, n_theta);
    [X_cyl, Y_cyl] = meshgrid([0, len], theta);
    Z_cyl = ones(size(X_cyl));
    
    % 원통 좌표 변환
    X_surf = start_x + X_cyl;
    Y_surf = radius * cos(Y_cyl);
    Z_surf = radius * sin(Y_cyl);
    
    % 표면 플롯
    surf(ax, X_surf, Y_surf, Z_surf, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 0.3);
    
    % 끝 캡 그리기
    theta_cap = linspace(0, 2*pi, n_theta);
    x_cap = zeros(1, numel(theta_cap));
    y_cap = radius * cos(theta_cap);
    z_cap = radius * sin(theta_cap);
    
    % 앞면 캡
    fill3(ax, start_x + x_cap, y_cap, z_cap, color, 'EdgeColor', 'none');
    
    % 뒷면 캡
    fill3(ax, start_x + len + x_cap, y_cap, z_cap, color, 'EdgeColor', 'none');
end

function drawTransition3D(ax, start_x, len, foreRadius, aftRadius, shape, color, n_theta)
    % 트랜지션 3D 그리기
    if len <= 0 || (foreRadius <= 0 && aftRadius <= 0)
        return;
    end
    
    % 형상에 따라 반경 프로파일 생성
    n_points = 30; % 세로 방향 점 개수
    x_local = linspace(0, len, n_points);
    
    % 기본 원뿔대 형상
    r_profile = zeros(1, n_points);
    
    % 형상 타입에 따른 반경 프로파일 결정
    switch lower(shape)
        case 'conical' % 기본 원뿔대
            r_profile = foreRadius + (aftRadius - foreRadius) * (x_local / len);
        case 'ogive' % 원호형 천이
            % OpenRocket 스타일 ogive 수식 적용
            rho = len / (1 - foreRadius/aftRadius);
            y_offset = sqrt(rho^2 - len^2) - rho + foreRadius;
            r_profile = sqrt(max(0, rho^2 - (len - x_local).^2)) + aftRadius - rho;
        case 'parabolic' % 포물선형 천이
            r_profile = foreRadius + (aftRadius - foreRadius) * (x_local / len).^2;
        otherwise % 기본값: 원뿔대
            r_profile = foreRadius + (aftRadius - foreRadius) * (x_local / len);
    end
    
    % 3D 표면 생성
    theta = linspace(0, 2*pi, n_theta);
    
    % 표면 좌표 계산
    [X_grid, T_grid] = meshgrid(x_local, theta);
    R_grid = repmat(r_profile, n_theta, 1);
    
    X_surf = start_x + X_grid;
    Y_surf = R_grid .* cos(T_grid);
    Z_surf = R_grid .* sin(T_grid);
    
    % 표면 플롯
    surf(ax, X_surf, Y_surf, Z_surf, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 0.3);
    
    % 끝 캡 그리기
    theta_cap = linspace(0, 2*pi, n_theta);
    
    % 앞면 캡
    y_cap_front = foreRadius * cos(theta_cap);
    z_cap_front = foreRadius * sin(theta_cap);
    fill3(ax, start_x + zeros(size(theta_cap)), y_cap_front, z_cap_front, color, 'EdgeColor', 'none');
    
    % 뒷면 캡
    y_cap_back = aftRadius * cos(theta_cap);
    z_cap_back = aftRadius * sin(theta_cap);
    fill3(ax, start_x + len + zeros(size(theta_cap)), y_cap_back, z_cap_back, color, 'EdgeColor', 'none');
end

function drawRing3D(ax, start_x, thickness, outerRadius, innerRadius, color, n_theta)
    % 센터링 링, 벌크헤드 등 원형 평판 3D 그리기
    if thickness <= 0 || outerRadius <= 0
        return;
    end
    
    % 기본값 처리
    if isnan(innerRadius) || innerRadius < 0
        innerRadius = 0;
    end
    
    % 원통 측면 그리기 (외부)
    theta = linspace(0, 2*pi, n_theta);
    [X_cyl, T_cyl] = meshgrid([0, thickness], theta);
    X_surf = start_x + X_cyl;
    Y_surf = outerRadius * cos(T_cyl);
    Z_surf = outerRadius * sin(T_cyl);
    
    surf(ax, X_surf, Y_surf, Z_surf, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
    
    % 내부 구멍이 있는 경우 내부 원통 측면 그리기
    if innerRadius > 0
        Y_inner = innerRadius * cos(T_cyl);
        Z_inner = innerRadius * sin(T_cyl);
        
        surf(ax, X_surf, Y_inner, Z_inner, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
    end
    
    % 앞뒤 원판 그리기
    theta_cap = linspace(0, 2*pi, n_theta);
    
    if innerRadius == 0
        % 내부 반경이 0인 경우 (벌크헤드) - 꽉 찬 원판
        n_r = 10; % 반경 방향 점 개수
        [R_grid, T_grid] = meshgrid(linspace(0, outerRadius, n_r), theta_cap);
        
        X_front = start_x + zeros(size(R_grid));
        Y_front = R_grid .* cos(T_grid);
        Z_front = R_grid .* sin(T_grid);
        
        X_back = start_x + thickness + zeros(size(R_grid));
        
        % 앞면과 뒷면 그리기
        surf(ax, X_front, Y_front, Z_front, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
        surf(ax, X_back, Y_front, Z_front, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
    else
        % 내부 반경이 0이 아닌 경우 (센터링 링) - 도넛 모양
        n_r = 10; % 반경 방향 점 개수
        [R_grid, T_grid] = meshgrid(linspace(innerRadius, outerRadius, n_r), theta_cap);
        
        X_front = start_x + zeros(size(R_grid));
        Y_front = R_grid .* cos(T_grid);
        Z_front = R_grid .* sin(T_grid);
        
        X_back = start_x + thickness + zeros(size(R_grid));
        
        % 앞면과 뒷면 그리기
        surf(ax, X_front, Y_front, Z_front, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
        surf(ax, X_back, Y_front, Z_front, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
    end
end

function drawSimpleComponent3D(ax, start_x, len, radius, color, n_theta)
    % 간단한 원통형 부품 그리기 (질량 부품, 낙하산 등)
    if len <= 0 || radius <= 0
        return;
    end
    
    % 원통형으로 단순화하여 표현
    drawBodyTube3D(ax, start_x, len, radius, color, n_theta);
    
    % 시각적 구분을 위한 패턴 추가 (예: 크로스 패턴)
    theta = linspace(0, 2*pi, n_theta);
    X_mid = start_x + len/2 + zeros(size(theta));
    Y_mid = radius * cos(theta);
    Z_mid = radius * sin(theta);
    
    % 중간 원 표시
    plot3(ax, X_mid, Y_mid, Z_mid, 'Color', color*0.8, 'LineWidth', 1.5);
    
    % 간단한 축 방향 라인 (구분용)
    for i = 1:4
        angle = (i-1) * pi/2;
        plot3(ax, [start_x, start_x+len], [radius*cos(angle), radius*cos(angle)], [radius*sin(angle), radius*sin(angle)], ...
            'Color', color*0.8, 'LineWidth', 1);
    end
end