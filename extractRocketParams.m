function rocketParams = extractRocketParams(xmlFilePath)
    fprintf('\n--- 파라미터 추출 시작 (하위 부품 포함) ---\n');

    % 결과를 저장할 셀 배열 초기화 (동적 추가 용이) - 최종적으로 구조체 배열로 변환
    rocketParams = {};

    % --- 1. XML 파일 읽기 ---
    try
        % Java XML 파서를 사용하여 XML 파일 읽기
        xmldata = xmlread(xmlFilePath);
        fprintf('XML 파일 "%s" 로드 성공 (파라미터 추출용).\n', xmlFilePath);
    catch ME
        fprintf(2, 'XML 파일 "%s" 로드 실패 (파라미터 추출용):\n%s\n', xmlFilePath, ME.message);
        rocketParams = struct([]); % 오류 시 빈 구조체 배열 반환
        return;
    end

    % --- 2. 최상위 부품 처리 로직 시작 ---
    try
        % 메인 부품 목록 노드 (일반적으로 <stage> 아래 <subcomponents>) 찾음
        % <rocket> -> <subcomponents> -> <stage> -> <subcomponents> 또는 <rocket> -> <stage> -> <subcomponents> 경로 지원
        stageSubcomponentsNode = findMainComponentListNode_ParamExtract(xmldata);

        if isempty(stageSubcomponentsNode)
            % 로켓 바로 아래에 subcomponents 없이 stage가 있는 경우를 대비하여 직접 찾아봅니다.
            rocketNode = xmldata.getElementsByTagName('rocket').item(0);
             if ~isempty(rocketNode)
                  stageNode = findDirectChildElement_ParamExtract(rocketNode, 'stage');
                  if ~isempty(stageNode)
                       % rocket -> stage 경로에서 subcomponents를 바로 찾습니다.
                       stageSubcomponentsNode = findDirectChildElement_ParamExtract(stageNode, 'subcomponents');
                  end
             end
        end

        if isempty(stageSubcomponentsNode)
            error('메인 부품 목록을 찾을 수 없습니다 (파라미터 추출용). XML 구조 확인 필요.');
        end

        % 재귀 함수 호출을 위한 초기값 설정
        % 최상위 레벨의 부모는 로켓 자체이며, 로켓의 시작점은 0, 길이는 0으로 가정
        initial_parent_abs_start_x = 0; % 스테이지의 시작점 (로켓 시작과 같음)
        % 주의: 최상위 Stage의 길이는 미리 알 수 없으므로 0을 전달합니다.
        initial_parent_len = 0;
        initial_sequential_base_x = 0; % 최상위 부품의 첫 순차적 시작 위치는 0
        initial_parent_radius = 0;     % 최상위 부품의 부모(로켓)의 반경은 0으로 간주

        % 최상위 부품 목록 처리 시작 (재귀 함수 호출)
        childNodes = stageSubcomponentsNode.getChildNodes;
        fprintf('메인 <subcomponents> 내에 처리할 노드 수: %d개\n', childNodes.getLength);

        processed_components_list = processNodeListRecursive_ParamExtract(...
            childNodes, ...
            initial_parent_abs_start_x, ...
            initial_parent_len, ...
            initial_sequential_base_x, ...
            initial_parent_radius);

        % 최종 결과를 구조체 배열로 변환
        if ~isempty(processed_components_list)
             % 셀 배열 내의 구조체들을 하나로 합침
             rocketParams = [processed_components_list{:}];
        else
            rocketParams = struct([]); % 처리된 부품이 없으면 빈 구조체 배열 반환
        end

        fprintf('파라미터 추출 완료. 총 %d개 부품 처리 (하위 부품 포함).\n', numel(rocketParams));

    catch ME
        fprintf(2, '\n*** 파라미터 추출 중 심각한 오류 발생 ***\n');
        fprintf(2, '오류 메시지: %s\n', ME.message);
         if ~isempty(ME.stack)
            % 첫 번째 스택 정보만 출력 (가장 최근의 오류 발생 지점)
            fprintf(2, '오류 위치: %s, %d번째 줄\n', ME.stack(1).name, ME.stack(1).line);
         end
         if ~isempty(ME.cause) && iscell(ME.cause) && ~isempty(ME.cause{1}) && isvalid(ME.cause{1})
             fprintf(2, '원인: %s\n', ME.cause{1}.message);
         end
        rocketParams = struct([]); % 오류 시 빈 구조체 배열 반환
    end
end % end extractRocketParams

% --- 재귀적으로 노드 리스트를 처리하는 함수 ---
function processed_list = processNodeListRecursive_ParamExtract(...
    nodeList, ...
    parent_abs_start_x, ... % 현재 레벨 노드들의 부모의 절대 시작 위치
    parent_len, ...         % 현재 레벨 노드들의 부모의 길이
    current_sequential_x_at_this_level, ... % 현재 레벨에서 다음 순차 부품의 예상 시작 위치 (이전 형제의 끝)
    current_parent_radius) % 현재 레벨 노드들의 부모의 반경

    processed_list = {}; % 현재 레벨에서 처리된 부품들을 저장할 셀 배열

    % 이 레벨에서의 다음 순차적 부품의 시작 위치를 추적하는 변수
    next_sequential_x_at_this_level = current_sequential_x_at_this_level;

    for i = 0:nodeList.getLength-1
        node = nodeList.item(i);

        % ELEMENT_NODE만 처리하고, 텍스트나 주석 노드는 무시
        if node.getNodeType == org.w3c.dom.Node.ELEMENT_NODE

            nodeName = char(node.getNodeName);

            % 현재 부품 정보 저장용 구조체 초기화 및 일반 정보 추출
            currentComponent = struct();

            % 기본 정보 추출
            currentComponent.Name = getNodeValue_ParamExtract(node, 'name', ['Unnamed ' nodeName]);
            currentComponent.Type = nodeName;
            currentComponent.Id = getNodeValue_ParamExtract(node, 'id', 'N/A'); % ID 추출 추가
            currentComponent.Finish = getNodeValue_ParamExtract(node, 'finish', 'N/A'); % Finish 추출 추가

            % Geometry, Material, Position, InstanceInfo, MotorMount, Subcomponents 구조체 초기화
            currentComponent.Geometry = struct();
            % 밀도 속성 값 저장 필드 추가 (타입 무관)
            currentComponent.Material = struct('Name', 'N/A', 'Type', 'N/A', 'DensityAttributeValue', NaN);
            currentComponent.Position = struct(...
                'AbsoluteStartX', NaN, ...
                'AbsoluteEndX', NaN, ...
                'Length', NaN, ...
                'EndRadius', current_parent_radius, ...
                'RefType', 'Sequential', ...
                'OverrideCGX', NaN, ...
                'IsCGOverridden', false);
            currentComponent.InstanceInfo = struct(...
                'Count', 1, ...
                'Separation', NaN, ...
                'RadialPosition', 0, ...
                'RadialDirection', 0, ...
                'AngleOffset', 0, ...
                'Rotation', 0);

            % MotorMount 구조체 초기화 (위치 정보 필드 추가)
            currentComponent.MotorMount = struct(...
                'HasMount', false, ...
                'MountAxialOffset', NaN, ... % 추가
                'MountPositionType', 'N/A', ... % 추가
                'MountPositionValue', NaN, ... % 추가
                'MotorOverhang', NaN, ... % 추가
                'Motor', struct('Type', 'N/A', 'Manufacturer', 'N/A', 'Designation', 'N/A', 'Digest', 'N/A', 'Diameter', NaN, 'Length', NaN, 'Delay', 'N/A'));

            % FinDetails, ParachuteDetails, ShoulderDetails 구조체 초기화 (필요시 채워짐)
            currentComponent.Geometry.FinDetails = struct(... % Trapezoid Fin 상세 정보 구조체
                'Cant', NaN, ...
                'FilletRadius', NaN, ...
                'CrossSection', 'N/A', ...
                'IsFlipped', false, ... % isflipped 속성 추가
                'FilletMaterial', struct('Name', 'N/A', 'Type', 'N/A', 'DensityAttributeValue', NaN)); % 필렛 재료 정보 구조체

            currentComponent.Geometry.ParachuteDetails = struct(... % Parachute 상세 정보 구조체
                'Diameter', NaN, ... % Geometry에 이미 있지만, ParachuteDetails에 같이 두는게 편할 수도.
                'Cd', NaN, ... % Geometry에 이미 있지만, Details에 같이 두는게 편할 수도.
                'LineCount', NaN, ... % 추가
                'LineLength', NaN, ... % 추가
                'LineMaterial', struct('Name', 'N/A', 'Type', 'N/A', 'DensityAttributeValue', NaN)); % 라인 재료 정보 구조체

            currentComponent.Geometry.ForeShoulder = struct(... % 전방 숄더 상세 정보 구조체 (Transition용)
                'Radius', NaN, ...
                'Length', NaN, ...
                'Thickness', NaN, ...
                'Capped', false); % 'foreshouldercapped' 속성 추가 필요

            currentComponent.Geometry.AftShoulder = struct(... % 후방 숄더 상세 정보 구조체 (Nosecone, Transition용)
                'Radius', NaN, ...
                'Length', NaN, ...
                'Thickness', NaN, ...
                'Capped', false); % 'aftshouldercapped' 속성 추가 필요

            % Mass는 계산 필요시 외부에서 수행
            currentComponent.Mass = NaN; % 질량 정보 초기화

            % 하위 부품 저장용 셀 배열 초기화
            currentComponent.Subcomponents = {};

            % fprintf('  처리 중: %s (%s)\n', compName, nodeName); % 디버깅 출력 (필요시 활성화)

            % --- 일반적인 인스턴스 및 3D 위치 정보 추출 ---
            % 대부분의 부품에 적용될 수 있는 인스턴스 정보
            instanceCount = str2double(getNodeValue_ParamExtract(node, 'instancecount', '1'));
            if ~isnan(instanceCount), currentComponent.InstanceInfo.Count = instanceCount; end
             % 인스턴스 간 간격
            instanceSeparation = str2double(getNodeValue_ParamExtract(node, 'instanceseparation', 'NaN'));
            if ~isnan(instanceSeparation), currentComponent.InstanceInfo.Separation = instanceSeparation; end
            % 3D 위치 오프셋 정보 (축 방향 위치와는 별개)
            radialPosition = str2double(getNodeValue_ParamExtract(node, 'radialposition', '0')); % 기본값 0 (축 위)
             if ~isnan(radialPosition), currentComponent.InstanceInfo.RadialPosition = radialPosition; end
            radialDirection = str2double(getNodeValue_ParamExtract(node, 'radialdirection', '0')); % 기본값 0
             if ~isnan(radialDirection), currentComponent.InstanceInfo.RadialDirection = radialDirection; end
            angleOffset = str2double(getNodeValue_ParamExtract(node, 'angleoffset', '0')); % 기본값 0
             if ~isnan(angleOffset), currentComponent.InstanceInfo.AngleOffset = angleOffset; end
            % 핀 등의 회전 정보
            rotation = str2double(getNodeValue_ParamExtract(node, 'rotation', '0')); % 기본값 0
             if ~isnan(rotation), currentComponent.InstanceInfo.Rotation = rotation; end

            % --- CG 오버라이드 정보 추출 ---
            overrideCGX = str2double(getNodeValue_ParamExtract(node, 'overridecgx', 'NaN'));
            if ~isnan(overrideCGX)
                 currentComponent.Position.OverrideCGX = overrideCGX;
            end
             % overridecg 속성 (boolean)
            overrideCGAttr = char(node.getAttribute('overridecg'));
            if strcmp(lower(overrideCGAttr), 'true')
                 currentComponent.Position.IsCGOverridden = true;
            elseif strcmp(lower(overrideCGAttr), 'false')
                 currentComponent.Position.IsCGOverridden = false; % 명시적으로 false인 경우
            end

            % --- 재료 정보 추출 (업그레이드: 밀도 속성 값 자체를 추출) ---
            matNode = getChildElementsByTagName_ParamExtract(node, 'material');
            if ~isempty(matNode) && matNode.getLength > 0
                 materialNode = matNode.item(0);
                 currentComponent.Material.Name = strtrim(char(materialNode.getTextContent)); % 재료 이름
                 currentComponent.Material.Type = char(materialNode.getAttribute('type')); % 재료 타입 (bulk, surface, line 등)
                 % 밀도 속성 값 자체를 추출 (타입 무관) - 해석은 외부에서
                 currentComponent.Material.DensityAttributeValue = getMaterialDensityAttribute_ParamExtract(materialNode, 'density', NaN); % 헬퍼 함수 이름 변경 및 입력 파라미터 수정
            end

            % --- 위치 정보 추출 및 절대 위치 계산 ---
            [absolute_start_x, position_type_desc] = getAbsoluteStartPosition_ParamExtract(...
                node, ...
                next_sequential_x_at_this_level, ...
                parent_abs_start_x, ...
                parent_len);

            currentComponent.Position.AbsoluteStartX = absolute_start_x;
            currentComponent.Position.RefType = position_type_desc;

            % 부품 길이 및 끝 반경 초기화 (아래 부품 타입별 처리에서 업데이트)
            component_len = 0;
            component_end_radius = current_parent_radius; % 기본적으로 부모 반경을 이어받음

            % --- 부품 타입별 상세 정보 추출 (업그레이드: 누락 항목 추가) ---
            switch nodeName
                case 'nosecone'
                    len = str2double(getNodeValue_ParamExtract(node, 'length', 'NaN'));
                    thk = str2double(getNodeValue_ParamExtract(node, 'thickness', 'NaN'));
                    aftR_str = getNodeValue_ParamExtract(node, 'aftradius', '');
                    aftR = parseRadius_ParamExtract(aftR_str, NaN); % Nosecone AftRadius default is NaN, not parent
                    shape = getNodeValue_ParamExtract(node, 'shape', 'N/A');
                    shapeParam = str2double(getNodeValue_ParamExtract(node, 'shapeparameter', 'NaN'));
                    shapeClippedAttr = char(node.getAttribute('shapeclipped')); % shapeclipped 속성 추가 추출
                    isFlippedAttr = char(node.getAttribute('isflipped')); % isflipped 속성 추가 추출

                    currentComponent.Geometry.Length = len;
                    currentComponent.Geometry.Thickness = thk;
                    currentComponent.Geometry.AftRadius = aftR;
                    currentComponent.Geometry.Shape = shape;
                    currentComponent.Geometry.ShapeParameter = shapeParam;
                    currentComponent.Geometry.ShapeClipped = strcmp(lower(shapeClippedAttr), 'true'); % 논리값 변환
                    currentComponent.Geometry.IsFlipped = strcmp(lower(isFlippedAttr), 'true'); % 논리값 변환

                    % 후방 숄더 정보 추출 (Nosecone)
                    currentComponent.Geometry.AftShoulder.Radius = str2double(getNodeValue_ParamExtract(node, 'aftshoulderradius', 'NaN'));
                    currentComponent.Geometry.AftShoulder.Length = str2double(getNodeValue_ParamExtract(node, 'aftshoulderlength', 'NaN'));
                    currentComponent.Geometry.AftShoulder.Thickness = str2double(getNodeValue_ParamExtract(node, 'aftshoulderthickness', 'NaN'));
                    aftShoulderCappedAttr = char(node.getAttribute('aftshouldercapped'));
                    currentComponent.Geometry.AftShoulder.Capped = strcmp(lower(aftShoulderCappedAttr), 'true');

                    % 길이 및 끝 반경 업데이트
                    if ~isnan(len), component_len = len; end
                    if ~isnan(aftR), component_end_radius = aftR; else component_end_radius = current_parent_radius; end % Use parent if AftRadius is auto/not specified

                case 'bodytube'
                    len = str2double(getNodeValue_ParamExtract(node, 'length', 'NaN'));
                    thk = str2double(getNodeValue_ParamExtract(node, 'thickness', 'NaN'));
                    radius_str = getNodeValue_ParamExtract(node, 'radius', '');
                    radius = parseRadius_ParamExtract(radius_str, current_parent_radius); % Bodytube Radius auto follows parent

                    currentComponent.Geometry.Length = len;
                    currentComponent.Geometry.Thickness = thk;
                    currentComponent.Geometry.Radius = radius;

                    % --- Motor Mount 정보 추출 (업그레이드: 위치 정보 추가) ---
                    motorMountNode = findDirectChildElement_ParamExtract(node, 'motormount');
                    if ~isempty(motorMountNode)
                         currentComponent.MotorMount.HasMount = true;

                         % 마운트 자체의 위치 정보 (부모 바디튜브 기준) 추출
                         % axialoffset 태그가 있다면 추출
                         currentComponent.MotorMount.MountAxialOffset = str2double(getNodeValue_ParamExtract(motorMountNode, 'axialoffset', 'NaN')); % axialoffset 태그 추출 추가

                         mountPosList = getChildElementsByTagName_ParamExtract(motorMountNode, 'position');
                         if ~isempty(mountPosList) && mountPosList.getLength > 0
                              mountPosNode = mountPosList.item(0);
                              currentComponent.MotorMount.MountPositionValue = str2double(strtrim(char(mountPosNode.getTextContent)));
                              currentComponent.MotorMount.MountPositionType = char(mountPosNode.getAttribute('type'));
                         end

                         % 모터 돌출 정보 추출
                         currentComponent.MotorMount.MotorOverhang = str2double(getNodeValue_ParamExtract(motorMountNode, 'overhang', 'NaN'));

                         % 모터 상세 정보 추출 (기존 로직 유지)
                         motorNode = findDirectChildElement_ParamExtract(motorMountNode, 'motor');
                         if ~isempty(motorNode)
                              % motor 태그의 속성들을 추출
                              currentComponent.MotorMount.Motor.Type = char(motorNode.getAttribute('type'));
                              currentComponent.MotorMount.Motor.Manufacturer = char(motorNode.getAttribute('manufacturer'));
                              currentComponent.MotorMount.Motor.Designation = char(motorNode.getAttribute('designation'));
                              currentComponent.MotorMount.Motor.Digest = char(motorNode.getAttribute('digest'));
                              currentComponent.MotorMount.Motor.Diameter = str2double(char(motorNode.getAttribute('diameter')));
                              currentComponent.MotorMount.Motor.Length = str2double(char(motorNode.getAttribute('length')));
                              currentComponent.MotorMount.Motor.Delay = char(motorNode.getAttribute('delay'));
                         end
                    end

                    % 길이 및 끝 반경 업데이트
                    if ~isnan(len), component_len = len; end
                    if ~isnan(radius), component_end_radius = radius; else component_end_radius = current_parent_radius; end % Use parent if Radius is auto/not specified

                case 'transition'
                    len = str2double(getNodeValue_ParamExtract(node, 'length', 'NaN'));
                    thk = str2double(getNodeValue_ParamExtract(node, 'thickness', 'NaN'));
                    foreR_str = getNodeValue_ParamExtract(node, 'foreradius', '');
                    aftR_str = getNodeValue_ParamExtract(node, 'aftradius', '');
                    foreR = parseRadius_ParamExtract(foreR_str, current_parent_radius); % Transition ForeRadius auto follows parent
                    aftR = parseRadius_ParamExtract(aftR_str, NaN); % Transition AftRadius default is NaN, not parent
                    shape = getNodeValue_ParamExtract(node, 'shape', 'N/A'); % 보통 'conical'

                    currentComponent.Geometry.Length = len;
                    currentComponent.Geometry.Thickness = thk;
                    currentComponent.Geometry.ForeRadius = foreR;
                    currentComponent.Geometry.AftRadius = aftR;
                    currentComponent.Geometry.Shape = shape;

                    % 전방 숄더 정보 추출 (Transition)
                    currentComponent.Geometry.ForeShoulder.Radius = str2double(getNodeValue_ParamExtract(node, 'foreshoulderradius', 'NaN'));
                    currentComponent.Geometry.ForeShoulder.Length = str2double(getNodeValue_ParamExtract(node, 'foeshoulderlength', 'NaN')); % 태그 이름 수정 (foeshoulderlength -> foreshoulderlength) - XML에 따라
                    currentComponent.Geometry.ForeShoulder.Thickness = str2double(getNodeValue_ParamExtract(node, 'foreshoulderthickness', 'NaN'));
                    foreShoulderCappedAttr = char(node.getAttribute('foreshouldercapped'));
                    currentComponent.Geometry.ForeShoulder.Capped = strcmp(lower(foreShoulderCappedAttr), 'true');

                    % 후방 숄더 정보 추출 (Transition)
                    currentComponent.Geometry.AftShoulder.Radius = str2double(getNodeValue_ParamExtract(node, 'aftshoulderradius', 'NaN'));
                    currentComponent.Geometry.AftShoulder.Length = str2double(getNodeValue_ParamExtract(node, 'aftshoulderlength', 'NaN'));
                    currentComponent.Geometry.AftShoulder.Thickness = str2double(getNodeValue_ParamExtract(node, 'aftshoulderthickness', 'NaN'));
                    aftShoulderCappedAttr = char(node.getAttribute('aftshouldercapped'));
                    currentComponent.Geometry.AftShoulder.Capped = strcmp(lower(aftShoulderCappedAttr), 'true');

                    % 길이 및 끝 반경 업데이트
                    if ~isnan(len), component_len = len; end
                    if ~isnan(aftR), component_end_radius = aftR;
                    % else component_end_radius = current_parent_radius; % Transition AftRadius is important, default not parent radius
                    end

                case 'trapezoidfinset' % 다른 핀 유형도 비슷한 파라미터를 가질 수 있습니다.
                    rootchord = str2double(getNodeValue_ParamExtract(node, 'rootchord', 'NaN'));
                    tipchord = str2double(getNodeValue_ParamExtract(node, 'tipchord', 'NaN'));
                    height = str2double(getNodeValue_ParamExtract(node, 'height', 'NaN'));
                    sweeplen = str2double(getNodeValue_ParamExtract(node, 'sweeplength', 'NaN'));
                    thk = str2double(getNodeValue_ParamExtract(node, 'thickness', 'NaN'));
                    cant = str2double(getNodeValue_ParamExtract(node, 'cant', 'NaN')); % cant 추출 추가
                    filletR = str2double(getNodeValue_ParamExtract(node, 'filletradius', 'NaN')); % filletradius 추출 추가
                    crossSection = getNodeValue_ParamExtract(node, 'crosssection', 'N/A'); % crosssection 추출 추가
                    isFlippedAttr = char(node.getAttribute('isflipped')); % isflipped 속성 추가 추출

                    currentComponent.Geometry.RootChord = rootchord;
                    currentComponent.Geometry.TipChord = tipchord;
                    currentComponent.Geometry.Height = height;
                    currentComponent.Geometry.Sweep = sweeplen;
                    currentComponent.Geometry.Thickness = thk;

                    % Fin 상세 정보 필드 채우기
                    currentComponent.Geometry.FinDetails.Cant = cant;
                    currentComponent.Geometry.FinDetails.FilletRadius = filletR;
                    currentComponent.Geometry.FinDetails.CrossSection = crossSection;
                    currentComponent.Geometry.FinDetails.IsFlipped = strcmp(lower(isFlippedAttr), 'true'); % 논리값 변환

                    % 필렛 재료 정보 추출 (업그레이드)
                    filletMatNode = findDirectChildElement_ParamExtract(node, 'filletmaterial');
                    if ~isempty(filletMatNode)
                         currentComponent.Geometry.FinDetails.FilletMaterial.Name = strtrim(char(filletMatNode.getTextContent));
                         currentComponent.Geometry.FinDetails.FilletMaterial.Type = char(filletMatNode.getAttribute('type'));
                         % 필렛 재료의 밀도 속성 추출
                         currentComponent.Geometry.FinDetails.FilletMaterial.DensityAttributeValue = getMaterialDensityAttribute_ParamExtract(filletMatNode, 'density', NaN);
                    end

                    component_len = 0; % 핀은 축방향 길이에 영향 없음
                    component_end_radius = current_parent_radius; % 핀은 외부에 부착, 부모 반경 유지

                case 'masscomponent'
                    % 질량 부품은 길이/반경이 Packed 정보로 주어짐
                    packedLen = str2double(getNodeValue_ParamExtract(node, 'packedlength', 'NaN'));
                    packedRadius = str2double(getNodeValue_ParamExtract(node, 'packedradius', 'NaN'));
                    mass_override = str2double(getNodeValue_ParamExtract(node, 'mass', 'NaN')); % Direct Mass

                    currentComponent.Geometry.PackedLength = packedLen;
                    currentComponent.Geometry.PackedRadius = packedRadius;

                    % MassComponent의 Mass는 XML의 mass 값을 사용 (가장 신뢰할 수 있는 값)
                    if ~isnan(mass_override)
                        currentComponent.Mass = mass_override;
                        % MassComponent의 Material 정보는 직접 질량 지정이므로 기본값 또는 Override로 설정
                        currentComponent.Material.Name = 'Direct Mass';
                        currentComponent.Material.Type = 'MassComponentOverride'; % 재료 타입도 오버라이드로 표시
                        currentComponent.Material.DensityAttributeValue = NaN; % 밀도는 무의미
                    else
                         % mass 태그가 없는 경우 (매우 드물지만), packed 정보와 재료 밀도로 계산 필요 (여기서는 NaN 유지)
                         % currentComponent.Mass = calculateMassFromPacked(packedLen, packedRadius, currentComponent.Material.DensityAttributeValue, currentComponent.Material.Type); % 계산 로직 추가 필요
                         currentComponent.Mass = NaN;
                    end

                    % 질량 부품은 길이가 Packed Length에 따름 (없으면 0)
                    if ~isnan(packedLen), component_len = packedLen; else component_len = 0; end

                    component_end_radius = current_parent_radius; % 부모 반경 그대로 유지

                case 'tubecoupler'
                    len = str2double(getNodeValue_ParamExtract(node, 'length', 'NaN'));
                    thk = str2double(getNodeValue_ParamExtract(node, 'thickness', 'NaN'));
                    outerR_str = getNodeValue_ParamExtract(node, 'outerradius', '');
                    outerR = parseRadius_ParamExtract(outerR_str, current_parent_radius); % Coupler OuterRadius auto follows parent

                    currentComponent.Geometry.Length = len;
                    currentComponent.Geometry.Thickness = thk;
                    currentComponent.Geometry.OuterRadius = outerR;

                    % 길이 업데이트
                    if ~isnan(len), component_len = len; end
                    component_end_radius = current_parent_radius; % 커플러는 내부에 위치, 부모 반경 유지

                case 'centeringring'
                    % Centering Ring의 길이는 두께를 의미
                    thk = str2double(getNodeValue_ParamExtract(node, 'length', 'NaN'));
                    outerR_str = getNodeValue_ParamExtract(node, 'outerradius', '');
                    innerR_str = getNodeValue_ParamExtract(node, 'innerradius', '');
                    outerR = parseRadius_ParamExtract(outerR_str, current_parent_radius); % Centering Ring OuterRadius auto follows parent
                    innerR = parseRadius_ParamExtract(innerR_str, NaN); % InnerRadius default is NaN

                    currentComponent.Geometry.Thickness = thk; % 'length' 태그의 값을 Thickness로 저장
                    currentComponent.Geometry.OuterRadius = outerR;
                    currentComponent.Geometry.InnerRadius = innerR;
                    % instancecount/instanceseparation는 InstanceInfo로 통일하여 추출됨.

                    % component_len은 두께
                    if ~isnan(thk), component_len = thk; else component_len = 0; end
                    component_end_radius = current_parent_radius; % 부모 반경 그대로 유지

                  case 'bulkhead'
                     % Bulkhead의 길이는 두께를 의미
                     thk = str2double(getNodeValue_ParamExtract(node, 'length', 'NaN'));
                     outerR_str = getNodeValue_ParamExtract(node, 'outerradius', '');
                     outerR = parseRadius_ParamExtract(outerR_str, current_parent_radius); % Bulkhead OuterRadius auto follows parent

                     currentComponent.Geometry.Thickness = thk; % 'length' 태그의 값을 Thickness로 저장
                     currentComponent.Geometry.OuterRadius = outerR;
                     % instancecount는 InstanceInfo로 통일하여 추출됨.

                     % component_len은 두께
                     if ~isnan(thk), component_len = thk; else component_len = 0; end
                     component_end_radius = current_parent_radius; % 부모 반경 그대로 유지

                  case 'parachute' % 스트리머(streamer) 등도 유사한 파라미터 가질 수 있음
                     % 낙하산은 질량 정보가 packedlength/radius와 함께 주어짐
                     dia = str2double(getNodeValue_ParamExtract(node, 'diameter', 'NaN'));
                     cd = str2double(getNodeValue_ParamExtract(node, 'cd', 'NaN'));
                     packedLen = str2double(getNodeValue_ParamExtract(node, 'packedlength', 'NaN'));
                     packedRadius = str2double(getNodeValue_ParamExtract(node, 'packedradius', 'NaN'));

                     currentComponent.Geometry.Diameter = dia; % Geometry에도 유지
                     currentComponent.Geometry.Cd = cd; % Geometry에도 유지
                     currentComponent.Geometry.PackedLength = packedLen;
                     currentComponent.Geometry.PackedRadius = packedRadius;

                     % Parachute 상세 정보 필드 채우기 (업그레이드: 라인 정보 추가)
                     currentComponent.Geometry.ParachuteDetails.Diameter = dia;
                     currentComponent.Geometry.ParachuteDetails.Cd = cd;
                     currentComponent.Geometry.ParachuteDetails.LineCount = str2double(getNodeValue_ParamExtract(node, 'linecount', 'NaN'));
                     currentComponent.Geometry.ParachuteDetails.LineLength = str2double(getNodeValue_ParamExtract(node, 'linelength', 'NaN'));

                     % 라인 재료 정보 추출 (업그레이드)
                     lineMatNode = findDirectChildElement_ParamExtract(node, 'linematerial');
                     if ~isempty(lineMatNode)
                          currentComponent.Geometry.ParachuteDetails.LineMaterial.Name = strtrim(char(lineMatNode.getTextContent));
                          currentComponent.Geometry.ParachuteDetails.LineMaterial.Type = char(lineMatNode.getAttribute('type'));
                          % 라인 재료의 밀도 속성 추출
                          currentComponent.Geometry.ParachuteDetails.LineMaterial.DensityAttributeValue = getMaterialDensityAttribute_ParamExtract(lineMatNode, 'density', NaN);
                     end

                     mass_override = str2double(getNodeValue_ParamExtract(node, 'mass', 'NaN')); % Direct Mass
                     % Parachute의 Mass는 XML의 mass 값을 사용 (자체 계산 안함)
                     if ~isnan(mass_override)
                        currentComponent.Mass = mass_override;
                        currentComponent.Material.Name = 'Direct Mass (Override)';
                        currentComponent.Material.Type = 'MassComponentOverride'; % 재료 타입도 오버라이드로 표시
                        currentComponent.Material.DensityAttributeValue = NaN; % 밀도는 무의미
                     else
                         % Mass가 명시되지 않은 경우, Packed Volume * Material Density(surface) + Line Mass (Line Count * Line Length * Line Material Density) 로 계산 필요 (여기서는 NaN 유지)
                         % currentComponent.Mass = calculateParachuteMass(packedLen, packedRadius, dia, lineCount, lineLength, currentComponent.Material, currentComponent.Geometry.ParachuteDetails.LineMaterial); % 계산 로직 추가 필요
                         currentComponent.Mass = NaN;
                     end

                     % component_len은 Packed Length에 따름 (없으면 0)
                     if ~isnan(packedLen), component_len = packedLen; else component_len = 0; end
                     component_end_radius = current_parent_radius; % 부모 반경 그대로 유지

                case 'stage'
                    % Stage 자체는 물리적인 부품이라기보다는 그룹핑의 의미가 강함
                    % 길이는 하위 부품에 의해 결정되므로 여기서 길이를 추출하거나 설정하지 않음 (하위 부품 합산 필요)
                    % 반경도 하위 부품의 반경을 따라가거나 결정됨 (하위 부품 합산 필요)
                    component_len = 0; % Stage 자체의 명시적 길이는 보통 0
                    component_end_radius = current_parent_radius; % Stage의 끝 반경은 하위 부품에 의해 결정, 일단 부모 반경 유지
                    % Stage의 Mass는 하위 부품의 합이므로 여기서 계산하지 않음
                    currentComponent.Mass = NaN; % 하위 부품 합산 필요

                    % Stage의 Material 정보는 의미 없을 수 있지만 XML에 있다면 추출
                    % (이 경우는 Stage 자체에 material 태그가 없으므로 기본값 유지)


                otherwise
                    % 지원되지 않는 부품 타입은 기본적인 길이, 반경 정보만 추출 시도
                    fprintf('Info: 부품 타입 "%s"에 대한 상세 파라미터 추출/질량 계산 로직 미구현. 기본 정보만 추출.\n', nodeName);
                    lenVal = str2double(getNodeValue_ParamExtract(node, 'length', '0')); % length 태그 값 사용
                    if ~isnan(lenVal), component_len = lenVal; else component_len = 0; end

                     % 기본 반경 정보 추출 시도 (radius, outerradius, fore/aftradius 중 하나)
                     tempR = NaN;
                     if isnan(tempR), tempR = parseRadius_ParamExtract(getNodeValue_ParamExtract(node, 'radius', ''), NaN); end
                     if isnan(tempR), tempR = parseRadius_ParamExtract(getNodeValue_ParamExtract(node, 'outerradius', ''), NaN); end
                     if isnan(tempR), tempR = parseRadius_ParamExtract(getNodeValue_ParamExtract(node, 'foreradius', ''), NaN); end
                     if isnan(tempR), tempR = parseRadius_ParamExtract(getNodeValue_ParamExtract(node, 'aftradius', ''), NaN); end

                     if ~isnan(tempR)
                         % Geometry 구조체에 모든 가능한 반경 필드를 기본값으로 추가하고 추출된 값 저장
                         % 기존 필드는 유지하고 발견된 값만 업데이트
                         if ~isfield(currentComponent.Geometry, 'Radius'), currentComponent.Geometry.Radius = NaN; end
                         if ~isfield(currentComponent.Geometry, 'OuterRadius'), currentComponent.Geometry.OuterRadius = NaN; end
                         if ~isfield(currentComponent.Geometry, 'ForeRadius'), currentComponent.Geometry.ForeRadius = NaN; end
                         if ~isfield(currentComponent.Geometry, 'AftRadius'), currentComponent.Geometry.AftRadius = NaN; end

                         % 어떤 반경 필드에 넣을지는 타입에 따라 다르지만, 여기서는 일단 radius에 저장 시도
                         % parseRadius_ParamExtract 호출 시 반환값만 사용하도록 수정
                         if ~isnan(parseRadius_ParamExtract(getNodeValue_ParamExtract(node, 'radius', ''), NaN))
                              currentComponent.Geometry.Radius = tempR;
                         elseif ~isnan(parseRadius_ParamExtract(getNodeValue_ParamExtract(node, 'outerradius', ''), NaN))
                              currentComponent.Geometry.OuterRadius = tempR;
                         elseif ~isnan(parseRadius_ParamExtract(getNodeValue_ParamExtract(node, 'foreradius', ''), NaN))
                              currentComponent.Geometry.ForeRadius = tempR;
                         elseif ~isnan(parseRadius_ParamExtract(getNodeValue_ParamExtract(node, 'aftradius', ''), NaN))
                              currentComponent.Geometry.AftRadius = tempR;
                         else % 기타 경우, Radius 필드에 저장 시도
                             currentComponent.Geometry.Radius = tempR;
                         end

                         component_end_radius = parseRadius_ParamExtract(getNodeValue_ParamExtract(node, 'aftradius', ''), tempR); % AftRadius 있으면 그거 사용, 없으면 추출된 tempR 사용
                     else
                         % 반경 정보를 찾지 못하면 부모 반경 유지
                         component_end_radius = current_parent_radius;
                     end
                    currentComponent.Mass = NaN; % 질량은 알 수 없음
                     % Material 정보는 위에서 이미 추출 시도됨
            end

            % 부품의 길이 및 끝 반경 정보를 구조체에 저장
            currentComponent.Position.Length = component_len;
            currentComponent.Position.AbsoluteEndX = absolute_start_x + component_len;
            % 다음 재귀 호출 시 부모의 끝 반경으로 사용될 값을 업데이트
            currentComponent.Position.EndRadius = component_end_radius;

            % --- 하위 부품 처리 (재귀 호출) ---
            subcomponentsNode = findDirectChildElement_ParamExtract(node, 'subcomponents');
            if ~isempty(subcomponentsNode)
                % 재귀 호출 시, 현재 부품의 정보(절대 시작, 길이, 끝 반경)를
                % 하위 부품에게는 '부모' 정보로 전달. 하위 부품의 초기 순차 기준점은 부모의 시작점.
                % 하위 부품의 부모 길이는 현재 부품의 component_len을 전달
                currentComponent.Subcomponents = processNodeListRecursive_ParamExtract(...
                    subcomponentsNode.getChildNodes, ...
                    absolute_start_x, ...      % 현재 부품의 절대 시작 위치 -> 하위 부품의 부모 시작
                    component_len, ...         % 현재 부품의 길이 -> 하위 부품의 부모 길이
                    absolute_start_x, ...      % 하위 부품의 초기 순차 기준점은 부모의 시작점
                    component_end_radius);     % 현재 부품의 끝 반경 -> 하위 부품의 부모 반경
            end

            % 현재 처리된 부품을 결과 리스트에 추가
            processed_list{end+1} = currentComponent;

            % --- 다음 순차적 부품의 시작 위치 업데이트 ---
            % 현재 부품의 절대 시작 위치와 길이를 더하여 다음 순차 부품의 시작 위치를 결정
            % 이 값은 다음 루프에서 getAbsolutePosition_ParamExtract 함수의 sequential_base_x로 전달됩니다.
            % Stage 타입은 명시적 길이가 0으로 처리되었으므로, 하위 부품이 끝나는 위치가 다음 순차 부품의 시작점이 됨.
            % Stage의 경우 Stage 내 하위 부품 중 가장 마지막 순차 부품의 AbsoluteEndX를 찾아야 Stage 이후의 다음 순차 부품 위치를 정확히 알 수 있습니다.
            % 이 코드는 추출 로직만 다루므로 이 계산은 외부에서 수행하는 것이 좋습니다.
            % 현재 로직 유지 (Stage는 component_len이 0이므로 이전 sequential_base_x를 그대로 사용하게 됩니다).
            next_sequential_x_at_this_level = absolute_start_x + component_len;

        end % if ELEMENT_NODE
    end % for nodeList loop
end % end processNodeListRecursive_ParamExtract

% --- Helper Functions ---
% Helper: 주어진 태그 이름의 자식 노드 값을 문자열로 가져옴
 function nodeValue = getNodeValue_ParamExtract(parentNode, tagName, defaultValue)
      nodeValue = defaultValue;
      try
          % getChildElementsByTagName은 재귀적으로 하위 노드를 찾으므로, 직계 자식만 찾으려면 getChildNodes 사용 필요
          % 여기서는 getChildElementsByTagName을 사용하여 해당 태그를 가진 첫 번째 노드를 찾습니다.
          nodeList = getChildElementsByTagName_ParamExtract(parentNode, tagName);
          if ~isempty(nodeList) && nodeList.getLength > 0 && ~isempty(nodeList.item(0).getFirstChild)
              nodeValue = strtrim(char(nodeList.item(0).getTextContent));
              % Note: Attributes are not retrieved by getTextContent. Use node.getAttribute() for attributes.
               % Unnamed 기본값인 경우는 빈 값도 유효할 수 있음
              if isempty(nodeValue) && ~strcmp(defaultValue, ['Unnamed ' tagName])
                  % 텍스트 내용이 비어있지만 태그는 존재하는 경우 (예: <tag></tag>), 기본값 사용
                  % nodeValue = defaultValue; % 기본값 대신 빈 문자열 반환
              end
          else
              % 태그 자체가 없거나 자식 노드가 없는 경우 기본값 사용
              nodeValue = defaultValue;
          end
      catch ME
           % 오류 무시 또는 로깅 가능 (현재는 무시)
           % fprintf('경고(ParamExtract): 태그 "%s" 값 추출 중 오류: %s\n', tagName, ME.message);
           nodeValue = defaultValue; % 오류 시 기본값 반환
      end
 end

 % Helper: 주어진 노드의 속성 값을 문자열로 가져옴 (새로운 헬퍼)
 % 이 헬퍼는 현재 코드에서 사용되지 않지만, 속성 추출에 유용할 수 있습니다.
 % function attrValue = getAttributeValue_ParamExtract(node, attributeName, defaultValue)
 %     attrValue = defaultValue;
 %     try
 %         if isempty(node), return; end
 %
 %         attr = node.getAttribute(attributeName);
 %         if ~isempty(attr)
 %             attrValue = char(attr);
 %         end
 %     catch ME
 %           attrValue = defaultValue; % 오류 시 기본값 반환
 %     end
 % end


 % Helper: 로켓 XML의 메인 스테이지 아래 <subcomponents> 노드를 찾음
 % <rocket> -> <subcomponents> -> <stage> -> <subcomponents> 또는 <rocket> -> <stage> -> <subcomponents> 경로 지원
 function subCompNode = findMainComponentListNode_ParamExtract(xmldata)
      subCompNode = []; % 결과 노드 초기화
      try
          rocketNode = xmldata.getElementsByTagName('rocket').item(0); if isempty(rocketNode), return; end

          % 1차 시도: <rocket> -> <subcomponents> -> <stage> -> <subcomponents> 경로 탐색 (예제 XML 구조)
          rocketSubNode = findDirectChildElement_ParamExtract(rocketNode, 'subcomponents');
          if ~isempty(rocketSubNode)
              mainStageNode_nested = findDirectChildElement_ParamExtract(rocketSubNode, 'stage');
              if ~isempty(mainStageNode_nested)
                   subCompNode = findDirectChildElement_ParamExtract(mainStageNode_nested, 'subcomponents');
                   if ~isempty(subCompNode), return; end % 찾았으면 바로 반환
              end
          end
          % 2차 시도: <rocket> -> <stage> -> <subcomponents> 경로 탐색 (일반적인 다단 로켓 구조)
           mainStageNode_direct = findDirectChildElement_ParamExtract(rocketNode, 'stage');
           if ~isempty(mainStageNode_direct)
                subCompNode = findDirectChildElement_ParamExtract(mainStageNode_direct, 'subcomponents');
                if ~isempty(subCompNode), return; end % 찾았으면 바로 반환
           end
          % 둘 다 찾지 못한 경우 subCompNode는 빈 상태로 반환
      catch ME
          fprintf('경고(ParamExtract): 메인 부품 목록 노드 검색 중 오류: %s\n', ME.message);
          subCompNode = []; % 오류 발생 시 빈 값 반환
      end
  end

  % Helper: 부모 노드의 직계 자식 중에서 특정 태그 이름의 첫 번째 ELEMENT 노드를 찾음
  function childElement = findDirectChildElement_ParamExtract(parentNode, tagName)
      childElement = []; % 결과 노드 초기화
      try
          if isempty(parentNode), return; end
          childNodes = parentNode.getChildNodes;
          for k = 0:childNodes.getLength-1
              child = childNodes.item(k);
              if child.getNodeType == org.w3c.dom.Node.ELEMENT_NODE && strcmp(char(child.getNodeName), tagName)
                  childElement = child;
                  return; % 찾으면 바로 반환
              end
          end
      catch ME
           % 오류 무시
           % fprintf('경고(ParamExtract): 직접 자식 요소 "%s" 검색 중 오류: %s\n', tagName, ME.message);
           childElement = []; % 오류 발생 시 빈 값 반환
      end
  end

  % Helper: 부모 노드 하위의 모든 깊이에서 특정 태그 이름의 모든 ELEMENT 노드를 NodeList로 가져옴
  function childElements = getChildElementsByTagName_ParamExtract(parentNode, tagName)
       childElements = []; % 결과 초기화
       try
          if isempty(parentNode), return; end
          % getElementsByTagName은 재귀적으로 하위 노드를 모두 찾습니다.
          childElements = parentNode.getElementsByTagName(tagName);
       catch ME
           % 오류 무시
           % fprintf('경고(ParamExtract): 자식 요소 "%s" 검색 중 오류: %s\n', tagName, ME.message);
           childElements = []; % 오류 발생 시 빈 값 반환
       end
  end

  % Helper: 부품 노드에서 위치 정보를 추출하고 절대 시작 위치를 계산
  function [start_x, position_type_desc] = getAbsoluteStartPosition_ParamExtract(...
      componentNode, ...
      sequential_base_x, ...      % 이전 형제 부품의 끝 위치 (순차적 위치의 기준)
      parent_abs_start_x, ...     % 부모 부품의 절대 시작 위치
      parent_len)                 % 부모 부품의 길이

      % 기본값: 순차적 위치 (이전 형제 부품 끝에 이어 붙는 위치)
      start_x = sequential_base_x;
      position_type_desc = 'Sequential (default)';

      try
          posList = getChildElementsByTagName_ParamExtract(componentNode, 'position');

          if ~isempty(posList) && posList.getLength > 0
              posNode = posList.item(0);
              posValueStr = strtrim(char(posNode.getTextContent));
              posValue = str2double(posValueStr);
              posType = char(posNode.getAttribute('type')); % position type 속성 읽기

              % position 값이 유효한 경우에만 처리
              if ~isnan(posValue)
                   % 최상위 Stage의 자식인지 판단 (부모 길이가 0에 가깝고 부모 시작점도 0에 가까우면)
                   is_top_level_stage_component = parent_len < 1e-9 && parent_abs_start_x < 1e-9;

                   % *** 위치 계산 로직 핵심 부분 ***
                   if strcmp(lower(posType), 'absolute')
                       % Absolute 위치는 항상 절대값 사용
                       start_x = posValue;
                       position_type_desc = sprintf('Absolute (%.4f)', posValue);
                   elseif is_top_level_stage_component
                       % 최상위 부품 중 Absolute가 아닌 경우, 순차적 위치만 사용 (값 무시)
                       start_x = sequential_base_x; % 이전 형제 끝에 바로 붙임
                       position_type_desc = sprintf('Top-Level Seq (type="%s", value=%.4f) - Value Ignored for Axial Pos', posType, posValue);
                   else % 하위 부품인 경우 (parent_len > 0) 표준 Relative 또는 Sequential + Offset 로직 적용
                        switch lower(posType)
                            case 'top'
                                start_x = parent_abs_start_x + posValue;
                                position_type_desc = sprintf('Relative to Parent Top (%.4f)', posValue);

                            case 'bottom'
                                start_x = parent_abs_start_x + parent_len + posValue;
                                position_type_desc = sprintf('Relative to Parent Bottom (%.4f)', posValue);

                            case 'middle'
                                start_x = parent_abs_start_x + parent_len / 2 + posValue;
                                position_type_desc = sprintf('Relative to Parent Middle (%.4f)', posValue);

                             case 'sequential'
                                 % 순차적 위치 (이전 부품 끝 기준) + 오프셋
                                 start_x = sequential_base_x + posValue; % 이전 형제 끝 + 오프셋
                                 position_type_desc = sprintf('Sequential + Offset (%.4f)', posValue);

                             otherwise
                                 % 알 수 없는 타입은 순차적 위치 + 오프셋으로 간주 (기본값 사용)
                                 start_x = sequential_base_x + posValue;
                                 position_type_desc = sprintf('Unknown type "%s" (%.4f) - Treated as Sequential + Offset', posType, posValue);
                        end % switch posType (하위 부품)
                   end % if absolute vs is_top_level_stage_component else
              else
                   % position node는 있지만 값이 NaN인 경우, 순차적 기본값 사용
                   start_x = sequential_base_x;
                   position_type_desc = sprintf('Position value invalid - Treated as Sequential');
                   fprintf('  경고(ParamExtract): 부품 "%s"의 위치 값("%s")이 유효하지 않습니다. 순차적으로 처리합니다.\n', ...
                        getNodeValue_ParamExtract(componentNode, 'name', 'N/A'), posValueStr);
              end % if ~isnan(posValue)
          end % if posList exists
          % If no position node exists, the initial start_x = sequential_base_x remains.

      catch ME
           % 오류 발생 시 순차적 기본값 사용
           fprintf('경고(ParamExtract): 부품 "%s" 위치 정보 추출/계산 중 오류: %s. 순차적 기준 위치(%.4f) 사용.\n', ...
               getNodeValue_ParamExtract(componentNode, 'name', 'N/A'), ME.message, sequential_base_x);
           start_x = sequential_base_x;
           position_type_desc = 'Error during calculation - Treated as Sequential';
      end
  end

  % Helper: 반경 문자열 ('0.068', 'auto 0.068', 'auto' 등)을 숫자로 파싱
  function radius = parseRadius_ParamExtract(radiusStr, default_radius)
      radius = default_radius; % 기본값 설정
      try
          if isempty(radiusStr), return; end

          radiusStr = strtrim(radiusStr);

          if startsWith(radiusStr, 'auto', 'IgnoreCase', true)
              parts = strsplit(radiusStr);
              if length(parts) >= 2
                  parsedVal = str2double(parts{2});
                  if ~isnan(parsedVal) && parsedVal >= 0
                       radius = parsedVal;
                  end
              end
          else
              parsedVal = str2double(radiusStr);
              if ~isnan(parsedVal) && parsedVal >= 0
                  radius = parsedVal;
              end
          end
      catch ME
           fprintf('경고(ParamExtract): 반경 문자열 "%s" 파싱 중 오류: %s. 기본값(%.4f) 사용.\n', radiusStr, ME.message, default_radius);
           radius = default_radius;
      end
  end

  % Helper: 주어진 material 노드에서 density 속성 값을 추출 (업그레이드, 헬퍼 이름 변경)
  % 이제 타입 무관하게 density 속성 값을 숫자로 변환합니다.
  function densityValue = getMaterialDensityAttribute_ParamExtract(materialNode, attributeName, defaultDensity)
    densityValue = defaultDensity; % 기본 밀도 설정
    try
        if isempty(materialNode), return; end
        % getAttributeValue_ParamExtract 헬퍼를 사용해도 되지만, 여기서는 density 속성만 추출하므로 직접 접근합니다.
        densityAttr = char(materialNode.getAttribute(attributeName));

        if ~isempty(densityAttr)
            parsedDensity = str2double(densityAttr);
            if ~isnan(parsedDensity) % NaN이 아니면 유효한 숫자 밀도
                densityValue = parsedDensity;
            end
        end
    catch ME
        densityValue = defaultDensity;
    end
end