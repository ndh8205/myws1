function components = feRP(xmlFilePath)
% feRP - Flatten and Extract Rocket Parameters
% XML 로켓 파일에서 기하학적 정보와 물리적 정보만 추출하여 평탄화된 구조체 배열 반환
%
% 입력:
%   xmlFilePath - 로켓 XML 파일 경로
% 출력:
%   components - 평탄화된 로켓 컴포넌트 정보 구조체 배열

    % 1. XML 파일 읽기
    try
        xmldata = xmlread(xmlFilePath);
        fprintf('XML 파일 "%s" 로드 성공\n', xmlFilePath);
    catch ME
        error('XML 파일 로드 실패: %s', ME.message);
    end
    
    % 2. 컴포넌트 정보를 저장할 배열 초기화
    components = struct([]);
    
    % 3. 모든 컴포넌트 노드 찾기
    compTypes = {'nosecone', 'bodytube', 'transition', 'trapezoidfinset', ...
                 'tubecoupler', 'centeringring', 'bulkhead', 'parachute', ...
                 'masscomponent'};
    
    % 4. 각 타입별로 모든 컴포넌트 추출
    for i = 1:length(compTypes)
        type = compTypes{i};
        nodes = xmldata.getElementsByTagName(type);
        
        for j = 0:nodes.getLength-1
            node = nodes.item(j);
            
            % 기본 컴포넌트 구조체 생성
            comp = struct();
            comp.Type = type;
            comp.Name = getNodeTextValue(node, 'name', ['Unnamed_' type '_' num2str(j+1)]);
            
            % 위치 정보 추출
            comp.Position = getPositionInfo(node);
            
            % 재질 정보 추출
            comp.Material = getMaterialInfo(node);
            
            % 컴포넌트 타입별 고유 정보 추출
            comp = extractTypeSpecificInfo(comp, node, type);
            
            % 배열에 추가
            if isempty(components)
                components = comp;
            else
                components(end+1) = comp;
            end
        end
    end
end

function pos = getPositionInfo(node)
    % 위치 정보 추출
    pos = struct();
    
    % 기본 위치 정보
    pos.X = NaN;  % 최종 계산 위치는 나중에 결정
    
    % axialoffset 정보 추출
    axialOffsetNode = findDirectChild(node, 'axialoffset');
    if ~isempty(axialOffsetNode)
        pos.AxialOffset = str2double(char(axialOffsetNode.getTextContent));
        pos.AxialMethod = char(axialOffsetNode.getAttribute('method'));
    else
        pos.AxialOffset = 0;
        pos.AxialMethod = 'bottom';
    end
    
    % position 정보 추출
    positionNode = findDirectChild(node, 'position');
    if ~isempty(positionNode)
        pos.PositionValue = str2double(char(positionNode.getTextContent));
        pos.PositionType = char(positionNode.getAttribute('type'));
    else
        pos.PositionValue = 0;
        pos.PositionType = 'bottom';
    end
    
    % 인스턴스 정보 추출 (핀셋 등에 사용)
    pos.InstanceCount = str2double(getNodeTextValue(node, 'instancecount', '1'));
    pos.Rotation = str2double(getNodeTextValue(node, 'rotation', '0'));
    pos.RadialPosition = str2double(getNodeTextValue(node, 'radialposition', '0'));
    pos.RadialDirection = str2double(getNodeTextValue(node, 'radialdirection', '0'));
end

function mat = getMaterialInfo(node)
    % 재질 정보 추출
    mat = struct();
    
    % 재질 노드 찾기
    materialNode = findDirectChild(node, 'material');
    if ~isempty(materialNode)
        mat.Name = strtrim(char(materialNode.getTextContent));
        mat.Type = char(materialNode.getAttribute('type'));
        mat.Density = str2double(char(materialNode.getAttribute('density')));
    else
        mat.Name = 'Unknown';
        mat.Type = 'unknown';
        mat.Density = NaN;
    end
    
    % 질량 정보 추출 (masscomponent 등에 사용)
    mass = str2double(getNodeTextValue(node, 'mass', 'NaN'));
    if ~isnan(mass)
        mat.Mass = mass;
    else
        mat.Mass = NaN;
    end
end

function comp = extractTypeSpecificInfo(comp, node, type)
    % 컴포넌트 타입별 고유 정보 추출
    switch type
        case 'nosecone'
            comp.Length = str2double(getNodeTextValue(node, 'length', 'NaN'));
            comp.Thickness = str2double(getNodeTextValue(node, 'thickness', 'NaN'));
            comp.Shape = getNodeTextValue(node, 'shape', 'conical');
            comp.ShapeParameter = str2double(getNodeTextValue(node, 'shapeparameter', '0'));
            comp.AftRadius = parseRadius(getNodeTextValue(node, 'aftradius', 'NaN'));
            
        case 'bodytube'
            comp.Length = str2double(getNodeTextValue(node, 'length', 'NaN'));
            comp.Thickness = str2double(getNodeTextValue(node, 'thickness', 'NaN'));
            comp.Radius = parseRadius(getNodeTextValue(node, 'radius', 'NaN'));
            
            % 모터 마운트 정보 추출
            motorMountNode = findDirectChild(node, 'motormount');
            if ~isempty(motorMountNode)
                comp.HasMotorMount = true;
                
                % 모터 정보
                motorNode = findDirectChild(motorMountNode, 'motor');
                if ~isempty(motorNode)
                    comp.Motor = struct();
                    comp.Motor.Manufacturer = char(motorNode.getAttribute('manufacturer'));
                    comp.Motor.Designation = char(motorNode.getAttribute('designation'));
                    comp.Motor.Diameter = str2double(char(motorNode.getAttribute('diameter')));
                    comp.Motor.Length = str2double(char(motorNode.getAttribute('length')));
                    comp.Motor.Delay = char(motorNode.getAttribute('delay'));
                end
            else
                comp.HasMotorMount = false;
            end
            
        case 'transition'
            comp.Length = str2double(getNodeTextValue(node, 'length', 'NaN'));
            comp.Thickness = str2double(getNodeTextValue(node, 'thickness', 'NaN'));
            comp.ForeRadius = parseRadius(getNodeTextValue(node, 'foreradius', 'NaN'));
            comp.AftRadius = parseRadius(getNodeTextValue(node, 'aftradius', 'NaN'));
            comp.Shape = getNodeTextValue(node, 'shape', 'conical');
            
        case 'trapezoidfinset'
            comp.RootChord = str2double(getNodeTextValue(node, 'rootchord', 'NaN'));
            comp.TipChord = str2double(getNodeTextValue(node, 'tipchord', 'NaN'));
            comp.Height = str2double(getNodeTextValue(node, 'height', 'NaN'));
            comp.Sweep = str2double(getNodeTextValue(node, 'sweeplength', 'NaN'));
            comp.Thickness = str2double(getNodeTextValue(node, 'thickness', 'NaN'));
            comp.FinCount = str2double(getNodeTextValue(node, 'fincount', '1'));
            comp.CrossSection = getNodeTextValue(node, 'crosssection', 'square');
            
        case 'tubecoupler'
            comp.Length = str2double(getNodeTextValue(node, 'length', 'NaN'));
            comp.Thickness = str2double(getNodeTextValue(node, 'thickness', 'NaN'));
            comp.OuterRadius = parseRadius(getNodeTextValue(node, 'outerradius', 'NaN'));
            
        case 'centeringring'
            comp.Thickness = str2double(getNodeTextValue(node, 'length', 'NaN'));
            comp.OuterRadius = parseRadius(getNodeTextValue(node, 'outerradius', 'NaN'));
            comp.InnerRadius = parseRadius(getNodeTextValue(node, 'innerradius', 'NaN'));
            
        case 'bulkhead'
            comp.Thickness = str2double(getNodeTextValue(node, 'length', 'NaN'));
            comp.OuterRadius = parseRadius(getNodeTextValue(node, 'outerradius', 'NaN'));
            
        case 'parachute'
            comp.Diameter = str2double(getNodeTextValue(node, 'diameter', 'NaN'));
            comp.Cd = str2double(getNodeTextValue(node, 'cd', 'NaN'));
            comp.PackedLength = str2double(getNodeTextValue(node, 'packedlength', 'NaN'));
            comp.PackedRadius = str2double(getNodeTextValue(node, 'packedradius', 'NaN'));
            comp.LineCount = str2double(getNodeTextValue(node, 'linecount', 'NaN'));
            comp.LineLength = str2double(getNodeTextValue(node, 'linelength', 'NaN'));
            
            % 라인 재질 추출
            lineMatNode = findDirectChild(node, 'linematerial');
            if ~isempty(lineMatNode)
                comp.LineMaterial = struct();
                comp.LineMaterial.Name = strtrim(char(lineMatNode.getTextContent));
                comp.LineMaterial.Density = str2double(char(lineMatNode.getAttribute('density')));
            end
            
        case 'masscomponent'
            comp.PackedLength = str2double(getNodeTextValue(node, 'packedlength', 'NaN'));
            comp.PackedRadius = str2double(getNodeTextValue(node, 'packedradius', 'NaN'));
            comp.Mass = str2double(getNodeTextValue(node, 'mass', 'NaN'));
    end
end

function nodeValue = getNodeTextValue(parentNode, tagName, defaultValue)
    % 주어진 태그 이름의 자식 노드 텍스트 값을 가져옴
    childNode = findDirectChild(parentNode, tagName);
    
    if ~isempty(childNode) && ~isempty(childNode.getTextContent())
        nodeValue = strtrim(char(childNode.getTextContent()));
    else
        nodeValue = defaultValue;
    end
end

function childElement = findDirectChild(parentNode, tagName)
    % 부모 노드의 직계 자식 중 특정 태그 이름을 가진 첫 번째 요소를 찾음
    childElement = [];
    
    if isempty(parentNode)
        return;
    end
    
    childNodes = parentNode.getChildNodes;
    for i = 0:childNodes.getLength-1
        node = childNodes.item(i);
        if node.getNodeType == node.ELEMENT_NODE && ...
           strcmp(char(node.getNodeName), tagName)
            childElement = node;
            return;
        end
    end
end

function radius = parseRadius(radiusStr)
    % 반경 문자열 해석 (auto 등 처리)
    radius = NaN;
    
    if isempty(radiusStr) || strcmp(radiusStr, 'NaN')
        return;
    end
    
    % 'auto XX' 형식 처리
    if contains(radiusStr, 'auto')
        parts = strsplit(radiusStr);
        if length(parts) >= 2
            radius = str2double(parts{2});
        end
    else
        % 직접 숫자 값
        radius = str2double(radiusStr);
    end
end