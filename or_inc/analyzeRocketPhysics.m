function analyzeRocketPhysics()
    % OpenRocket XML 파일에서 물리량을 파싱하여 표시하는 함수
    fprintf('OpenRocket Physics Analyzer V1.1 (개선버전)\n');
    fprintf('===================================================\n');
    xmlFilePath = 'rocket.xml';
    % --- 1. XML 파일 읽기 ---
    try
        xmldata = xmlread(xmlFilePath);
        fprintf('XML 파일 "%s" 로드 성공.\n\n', xmlFilePath);
    catch ME
        fprintf('XML 파일 "%s" 로드 실패:\n%s\n', xmlFilePath, ME.message);
        return;
    end

    % --- 2. 기본 로켓 정보 추출 ---
    [rocketName, designer] = getRocketInfo(xmldata);
    fprintf('로켓 이름: %s\n', rocketName);
    fprintf('설계자: %s\n\n', designer);
    
    % --- 3. 부품 분석 시작 ---
    fprintf('--- 물리량 분석 시작 ---\n');
    
    % 모든 컴포넌트의 질량, 위치, 관성 값을 계산하기 위한 배열
    components = struct('name', {}, 'type', {}, 'mass', {}, 'position', {}, 'length', {}, 'radius', {}, 'cm', {});
    
    % 재귀적으로 전체 XML 트리에서 모든 컴포넌트 파싱
    fprintf('\n주요 컴포넌트 구조 추출:\n');
    
    % 각 메인 컴포넌트 직접 처리 (재귀 대신)
    % 로켓 노드 찾기
    rocketNode = xmldata.getElementsByTagName('rocket').item(0);
    if isempty(rocketNode)
        fprintf('오류: <rocket> 노드를 찾을 수 없음\n');
        return;
    end
    
    % 스테이지 찾기
    stageNodes = rocketNode.getElementsByTagName('stage');
    if stageNodes.getLength == 0
        fprintf('오류: <stage> 노드를 찾을 수 없음\n');
        return;
    end
    
    % 최상위 컴포넌트 찾기 (nosecone, bodytube 등)
    for i = 0:stageNodes.getLength-1
        stageNode = stageNodes.item(i);
        stageName = getNodeValue(stageNode, 'name', 'Unnamed Stage');
        fprintf('스테이지 발견: %s\n', stageName);
        
        % 스테이지 내 subcomponents 노드 찾기
        subcompNodes = stageNode.getElementsByTagName('subcomponents');
        if subcompNodes.getLength > 0
            % 첫 번째 subcomponents 내 컴포넌트 처리
            subcompNode = subcompNodes.item(0);
            childNodes = subcompNode.getChildNodes;
            
            for j = 0:childNodes.getLength-1
                childNode = childNodes.item(j);
                if childNode.getNodeType ~= 1 % ELEMENT_NODE = 1
                    continue;
                end
                
                nodeName = char(childNode.getNodeName);
                compName = getNodeValue(childNode, 'name', ['Unnamed ' nodeName]);
                
                % 컴포넌트 물리적 속성 추출
                comp_len = extractComponentLength(childNode);
                comp_mass = extractComponentMass(childNode);
                comp_radius = extractComponentRadius(childNode);
                
                % 위치는 이전 부품 끝 기준
                position = 0;
                if numel(components) > 0
                    lastComp = components(end);
                    position = lastComp.position + lastComp.length;
                end
                
                % 부품의 위치 태그 확인
                posNodes = childNode.getElementsByTagName('position');
                if posNodes.getLength > 0
                    posNode = posNodes.item(0);
                    posValue = str2double(posNode.getTextContent);
                    posType = char(posNode.getAttribute('type'));
                    
                    if strcmp(posType, 'absolute')
                        position = posValue;
                    end
                end
                
                % 무게중심 계산 (부품 길이의 중앙 가정)
                cm_pos = position + comp_len/2;
                
                % 부품 정보 저장
                component = struct('name', compName, 'type', nodeName, ...
                                  'mass', comp_mass, 'position', position, ...
                                  'length', comp_len, 'radius', comp_radius, ...
                                  'cm', cm_pos);
                
                components(end+1) = component;
                
                fprintf('%s (%s): 위치=%.3f m, 길이=%.3f m, 질량=%.3f kg\n', ...
                        compName, nodeName, position, comp_len, comp_mass);
                
                % 자식 masscomponent 추가
                massNodes = childNode.getElementsByTagName('masscomponent');
                for k = 0:massNodes.getLength-1
                    massNode = massNodes.item(k);
                    massName = getNodeValue(massNode, 'name', 'Unnamed Mass');
                    massVal = str2double(getNodeValue(massNode, 'mass', '0'));
                    
                    if massVal > 0
                        components(end+1) = struct('name', massName, 'type', 'masscomponent', ...
                                               'mass', massVal, 'position', position, ...
                                               'length', 0, 'radius', 0, ...
                                               'cm', position);
                        
                        fprintf('  > 질량 부품: %s - 위치: %.3f m, 질량: %.3f kg\n', ...
                                massName, position, massVal);
                    end
                end
            end
        end
    end
    
    % 모터 질량 추가 (AeroTech M2400T는 약 9kg)
    motorMass = extractMotorInfo(xmldata);
    if motorMass > 0
        fprintf('\n모터 (AeroTech M2400T) 정보: 질량 약 %.1f kg\n', motorMass);
        % 모터 위치는 엔진 마운트가 있는 튜브의 후방으로 가정
        engineTubeIdx = -1;
        for i = 1:length(components)
            if contains(lower(components(i).name), 'engine') || contains(lower(components(i).type), 'motormount')
                engineTubeIdx = i;
                break;
            end
        end
        
        if engineTubeIdx > 0
            motorPosition = components(engineTubeIdx).position + components(engineTubeIdx).length * 0.7;
            components(end+1) = struct('name', 'AeroTech M2400T', 'type', 'motor', ...
                                    'mass', motorMass, 'position', motorPosition, ...
                                    'length', 0.597, 'radius', 0.049, 'cm', motorPosition);
        else
            components(end+1) = struct('name', 'AeroTech M2400T', 'type', 'motor', ...
                                    'mass', motorMass, 'position', 0, ...
                                    'length', 0.597, 'radius', 0.049, 'cm', 0.299);
        end
    end
    
    % 물리량 계산
    totalMass = 0;
    totalMomentum = 0;
    Ixx = 0;
    Iyy = 0;
    Izz = 0;
    
    % 질량, 무게중심 계산
    for i = 1:numel(components)
        totalMass = totalMass + components(i).mass;
        totalMomentum = totalMomentum + components(i).mass * components(i).cm;
    end
    
    % 전체 무게중심 계산
    centerOfMass = 0;
    if totalMass > 0
        centerOfMass = totalMomentum / totalMass;
    end
    
    % 관성 모멘트 계산 (무게중심 기준)
    for i = 1:numel(components)
        % 기본 관성 모멘트 (단순화된 실린더 형태 가정)
        mass = components(i).mass;
        radius = components(i).radius;
        comp_len = components(i).length;
        
        % 컴포넌트 무게중심
        cm = components(i).cm;
        
        % 롤 방향 관성 모멘트 (x축)
        if radius > 0
            Ixx = Ixx + mass * radius^2 / 2;
        end
        
        % 피치/요 방향 관성 모멘트 (y/z축)
        if radius > 0 && comp_len > 0
            I_cm = mass * (3*radius^2 + comp_len^2) / 12;
            d_squared = (cm - centerOfMass)^2;
            Iyy = Iyy + I_cm + mass * d_squared;
            Izz = Izz + I_cm + mass * d_squared;
        end
    end
    
    % --- 4. 결과 출력 ---
    fprintf('\n--- 로켓 물리량 분석 결과 ---\n');
    fprintf('총 질량: %.3f kg\n', totalMass);
    fprintf('무게중심 위치(길이 방향): %.3f m\n', centerOfMass);
    fprintf('관성 모멘트 (근사값):\n');
    fprintf('  Ixx (Roll): %.6f kg·m²\n', Ixx);
    fprintf('  Iyy (Pitch): %.6f kg·m²\n', Iyy);
    fprintf('  Izz (Yaw): %.6f kg·m²\n', Izz);
    
    % --- 5. 질량 분포 분석 ---
    fprintf('\n--- 질량 분포 분석 ---\n');
    
    % 컴포넌트 타입별 질량 합산
    typeGroups = struct('type', {}, 'mass', {});
    
    for i = 1:length(components)
        type = components(i).type;
        mass = components(i).mass;
        
        % 이미 존재하는 타입인지 확인
        found = false;
        for j = 1:length(typeGroups)
            if strcmp(typeGroups(j).type, type)
                typeGroups(j).mass = typeGroups(j).mass + mass;
                found = true;
                break;
            end
        end
        
        % 새 타입이면 추가
        if ~found
            typeGroups(end+1) = struct('type', type, 'mass', mass);
        end
    end
    
    % 주요 질량 항목 표시
    [~, sortIdx] = sort([typeGroups.mass], 'descend');
    fprintf('컴포넌트 타입별 질량 분포:\n');
    for i = 1:length(sortIdx)
        idx = sortIdx(i);
        fprintf('  - %s: %.3f kg (%.1f%%)\n', ...
            typeGroups(idx).type, typeGroups(idx).mass, typeGroups(idx).mass/totalMass*100);
    end
    
    % 주요 개별 컴포넌트 질량 표시
    [~, sortIdx] = sort([components.mass], 'descend');
    topN = min(10, length(sortIdx));
    fprintf('\n질량 상위 %d개 컴포넌트:\n', topN);
    for i = 1:topN
        idx = sortIdx(i);
        fprintf('  - %s (%s): %.3f kg (%.1f%%)\n', ...
            components(idx).name, components(idx).type, ...
            components(idx).mass, components(idx).mass/totalMass*100);
    end
    
    % --- 6. 물리량 시각화 ---
    figure('Name', sprintf('Rocket Physics: %s', rocketName), 'NumberTitle', 'off');
    
    % 타입별 질량 파이 차트
    subplot(2, 2, 1);
    masses = [typeGroups.mass];
    labels = {typeGroups.type};
    
    % 매우 작은 질량 그룹화 (가독성 향상)
    threshold = totalMass * 0.03; % 전체의 3% 미만인 항목
    otherMass = 0;
    finalLabels = {};
    finalMasses = [];
    
    for i = 1:numel(masses)
        if masses(i) > threshold
            finalLabels{end+1} = labels{i};
            finalMasses(end+1) = masses(i);
        else
            otherMass = otherMass + masses(i);
        end
    end
    
    if otherMass > 0
        finalLabels{end+1} = 'Others';
        finalMasses(end+1) = otherMass;
    end
    
    pie(finalMasses, finalLabels);
    title('컴포넌트 타입별 질량 분포');
    
    % 무게중심 위치 표시 (축 방향)
    subplot(2, 2, 2);
    
    % 주요 컴포넌트만 표시
    [sortedMasses, sortIdx] = sort([components.mass], 'descend');
    topN = min(8, length(sortIdx));
    topIdx = sortIdx(1:topN);
    
    compNames = {components(topIdx).name};
    compPositions = [components(topIdx).position];
    compMasses = [components(topIdx).mass];
    
    barh(1:topN, compPositions, 'b');
    hold on;
    plot([centerOfMass, centerOfMass], ylim, 'r--', 'LineWidth', 2);
    hold off;
    xlabel('축 방향 위치 (m)');
    title('주요 부품 위치 및 무게중심');
    set(gca, 'YTick', 1:topN);
    set(gca, 'YTickLabel', compNames);
    
    % 축방향 질량 분포 (히스토그램)
    subplot(2, 2, 3);
    edges = linspace(0, max([components.position]) + max([components.length]), 20);
    massDistribution = zeros(size(edges)-[0,1]);
    
    for i = 1:length(components)
        if components(i).length > 0
            startPos = components(i).position;
            endPos = startPos + components(i).length;
            mass = components(i).mass;
            
            % 길이에 따라 질량 분포
            for j = 1:length(edges)-1
                if startPos <= edges(j+1) && endPos >= edges(j)
                    % 겹치는 부분 계산
                    overlapStart = max(startPos, edges(j));
                    overlapEnd = min(endPos, edges(j+1));
                    overlapFraction = (overlapEnd - overlapStart) / (endPos - startPos);
                    massDistribution(j) = massDistribution(j) + mass * overlapFraction;
                end
            end
        else
            % 점 질량인 경우
            pos = components(i).position;
            for j = 1:length(edges)-1
                if pos >= edges(j) && pos < edges(j+1)
                    massDistribution(j) = massDistribution(j) + components(i).mass;
                    break;
                end
            end
        end
    end
    
    bar(edges(1:end-1) + diff(edges)/2, massDistribution);
    hold on;
    plot([centerOfMass, centerOfMass], ylim, 'r--', 'LineWidth', 2);
    hold off;
    xlabel('축 방향 위치 (m)');
    ylabel('질량 분포 (kg)');
    title('축 방향 질량 분포');
    
    % 전체 물리량 요약 표
    subplot(2, 2, 4);
    summaryData = {'총 질량 (kg)', totalMass;
                  '무게중심 (m)', centerOfMass;
                  '관성 모멘트 Ixx (kg·m²)', Ixx;
                  '관성 모멘트 Iyy (kg·m²)', Iyy;
                  '관성 모멘트 Izz (kg·m²)', Izz};
    uitable('Data', summaryData, 'ColumnName', {'속성', '값'}, ...
            'RowName', [], 'Position', [50, 50, 300, 150]);
    title('로켓 물리량 요약');
    axis off;
    
    % --- 구성품 테이블 생성 (새 창) ---
    figure('Name', '로켓 구성품 목록', 'NumberTitle', 'off');
    componentTable = table({components.name}', {components.type}', [components.mass]', ...
                           [components.position]', [components.length]', [components.radius]', ...
                           'VariableNames', {'이름', '타입', '질량_kg', '시작위치_m', '길이_m', '반경_m'});
    uitable('Data', table2cell(componentTable), 'ColumnName', componentTable.Properties.VariableNames, ...
            'RowName', [], 'Position', [20, 20, 800, 400]);
    axis off;
end

% =========================================================================
% 헬퍼 함수
% =========================================================================

function components = parseComponentsRecursively(xmldata, components, depth)
    % XML 구조를 재귀적으로 순회하며 모든 컴포넌트 파싱
    if depth == 0
        % 최상위에서는 로켓 노드부터 시작
        rocketNode = xmldata.getElementsByTagName('rocket').item(0);
        if isempty(rocketNode)
            fprintf('오류: <rocket> 노드를 찾을 수 없음\n');
            return;
        end
        
        % 로켓의 스테이지 찾기
        stageNodes = getElementsByTagNameRecursively(rocketNode, 'stage');
        if isempty(stageNodes) || stageNodes.getLength == 0
            fprintf('오류: <stage> 노드를 찾을 수 없음\n');
            return;
        end
        
        % 각 스테이지 처리
        for i = 0:stageNodes.getLength-1
            stageNode = stageNodes.item(i);
            stageName = getNodeValue(stageNode, 'name', 'Unnamed Stage');
            fprintf('스테이지 발견: %s\n', stageName);
            
            % 재귀적으로 스테이지 내 부품 처리
            components = parseComponentNodes(stageNode, components, depth + 1, 0);
        end
    end
end

function components = parseComponentNodes(parentNode, components, depth, parentPosition)
    % 부품 노드 처리 (재귀적)
    subcompNodes = parentNode.getElementsByTagName('subcomponents');
    if isempty(subcompNodes) || subcompNodes.getLength == 0
        return;
    end
    
    % 각 subcomponents 노드에 대해
    for sc = 0:subcompNodes.getLength-1
        subcompNode = subcompNodes.item(sc);
        
        % 직계 자식인지 확인
        if ~isDirectChild(parentNode, subcompNode)
            continue;
        end
        
        % subcomponents 내 모든 부품 처리
        childNodes = subcompNode.getChildNodes;
        for i = 0:childNodes.getLength-1
            childNode = childNodes.item(i);
            if childNode.getNodeType ~= org.w3c.dom.Node.ELEMENT_NODE
                continue;
            end
            
            nodeName = char(childNode.getNodeName);
            compName = getNodeValue(childNode, 'name', ['Unnamed ' nodeName]);
            
            % 부품 위치 계산
            [absolute_start_x, ~] = getAbsoluteStartPosition(childNode, parentPosition);
            
            % 부품 길이 및 질량 추출
            component_len = extractComponentLength(childNode);
            component_mass = extractComponentMass(childNode);
            component_radius = extractComponentRadius(childNode);
            
            % 부품 무게중심 계산 (단순화: 부품 중앙으로 가정)
            component_cm = absolute_start_x;
            if component_len > 0
                component_cm = absolute_start_x + component_len/2;
            end
            
            % 특수 처리: masscomponent의 경우
            if strcmp(nodeName, 'masscomponent')
                packedlength = str2double(getNodeValue(childNode, 'packedlength', '0'));
                if packedlength > 0
                    component_len = packedlength;
                    component_cm = absolute_start_x + component_len/2;
                end
            end
            
            % 부품 정보 추가
            newComponent = struct('name', compName, 'type', nodeName, ...
                                  'mass', component_mass, 'position', absolute_start_x, ...
                                  'length', component_len, 'radius', component_radius, ...
                                  'cm', component_cm);
            
            % 깊이에 따른 들여쓰기로 부품 출력
            indent = repmat('  ', 1, depth);
            fprintf('%s%s (%s): 위치=%.3f m, 길이=%.3f m, 질량=%.3f kg\n', ...
                    indent, compName, nodeName, absolute_start_x, component_len, component_mass);
            
            components(end+1) = newComponent;
            
            % 재귀적으로 하위 컴포넌트 처리
            components = parseComponentNodes(childNode, components, depth + 1, absolute_start_x + component_len);
        end
    end
end

function components = parseComponentNodes(parentNode, components, depth, parentPosition)
    % 부품 노드 처리 (재귀적)
    subcompNodes = parentNode.getElementsByTagName('subcomponents');
    if isempty(subcompNodes) || subcompNodes.getLength == 0
        return;
    end
    
    % 각 subcomponents 노드에 대해
    for sc = 0:subcompNodes.getLength-1
        subcompNode = subcompNodes.item(sc);
        
        % 직계 자식인지 확인
        if ~isDirectChild(parentNode, subcompNode)
            continue;
        end
        
        % subcomponents 내 모든 부품 처리
        childNodes = subcompNode.getChildNodes;
        for i = 0:childNodes.getLength-1
            childNode = childNodes.item(i);
            if childNode.getNodeType ~= org.w3c.dom.Node.ELEMENT_NODE
                continue;
            end
            
            nodeName = char(childNode.getNodeName);
            compName = getNodeValue(childNode, 'name', ['Unnamed ' nodeName]);
            
            % 부품 위치 계산
            [absolute_start_x, ~] = getAbsoluteStartPosition(childNode, parentPosition);
            
            % 부품 길이 및 질량 추출
            component_len = extractComponentLength(childNode);
            component_mass = extractComponentMass(childNode);
            component_radius = extractComponentRadius(childNode);
            
            % 부품 무게중심 계산 (단순화: 부품 중앙으로 가정)
            component_cm = absolute_start_x;
            if component_len > 0
                component_cm = absolute_start_x + component_len/2;
            end
            
            % 특수 처리: masscomponent의 경우
            if strcmp(nodeName, 'masscomponent')
                packedlength = str2double(getNodeValue(childNode, 'packedlength', '0'));
                if packedlength > 0
                    component_len = packedlength;
                    component_cm = absolute_start_x + component_len/2;
                end
            end
            
            % 부품 정보 추가
            newComponent = struct('name', compName, 'type', nodeName, ...
                                  'mass', component_mass, 'position', absolute_start_x, ...
                                  'length', component_len, 'radius', component_radius, ...
                                  'cm', component_cm);
            
            % 깊이에 따른 들여쓰기로 부품 출력
            indent = repmat('  ', 1, depth);
            fprintf('%s%s (%s): 위치=%.3f m, 길이=%.3f m, 질량=%.3f kg\n', ...
                    indent, compName, nodeName, absolute_start_x, component_len, component_mass);
            
            components(end+1) = newComponent;
            
            % 재귀적으로 하위 컴포넌트 처리
            components = parseComponentNodes(childNode, components, depth + 1, absolute_start_x + component_len);
        end
    end
end

function isDirectChildNode = isDirectChild(parentNode, childNode)
    % 주어진 노드가 부모 노드의 직계 자식인지 확인
    isDirectChildNode = false;
    childNodes = parentNode.getChildNodes;
    
    for i = 0:childNodes.getLength-1
        node = childNodes.item(i);
        if node == childNode
            isDirectChildNode = true;
            return;
        end
    end
end

function nodeList = getElementsByTagNameRecursively(parentNode, tagName)
    % 재귀적으로 태그 이름으로 노드 검색
    nodeList = parentNode.getElementsByTagName(tagName);
end

function [rocketName, designer] = getRocketInfo(xmldata)
    % 로켓 기본 정보 추출
    rocketName = 'Unknown Rocket'; 
    designer = 'Unknown';
    rocketNode = xmldata.getElementsByTagName('rocket').item(0);
    if isempty(rocketNode), return; end
    rocketName = getNodeValue(rocketNode, 'name', 'Name Not Found');
    designer = getNodeValue(rocketNode, 'designer', 'Unknown');
end

function nodeValue = getNodeValue(parentNode, tagName, defaultValue)
    % 노드 값 읽기
    nodeList = parentNode.getElementsByTagName(tagName);
    if nodeList.getLength > 0 && ~isempty(nodeList.item(0).getFirstChild)
        nodeValue = char(nodeList.item(0).getTextContent);
    else 
        nodeValue = defaultValue; 
    end
end

function [start_x, position_type_desc] = getAbsoluteStartPosition(componentNode, default_start_x)
    % 부품의 절대 시작 위치 계산
    start_x = default_start_x;
    position_type_desc = 'Sequential';
    
    posList = componentNode.getElementsByTagName('position');
    if posList.getLength > 0
        posNode = posList.item(0);
        posValue = str2double(posNode.getTextContent());
        posType = char(posNode.getAttribute('type'));
        
        switch posType
            case 'absolute'
                start_x = posValue;
                position_type_desc = sprintf('Absolute (%.4f)', posValue);
            case 'top'
                start_x = default_start_x + posValue;
                position_type_desc = sprintf('Top + %.4f', posValue);
            case 'bottom'
                % bottom은 부모 길이를 알아야 정확히 계산 가능
                % 여기서는 단순화해서 default_start_x를 그대로 사용
                start_x = default_start_x + posValue;
                position_type_desc = sprintf('Bottom + %.4f', posValue);
            case 'middle'
                % middle도 부모 길이의 중간점을 알아야 함
                start_x = default_start_x + posValue;
                position_type_desc = sprintf('Middle + %.4f', posValue);
            otherwise
                start_x = default_start_x;
                position_type_desc = sprintf('Unknown type "%s" - Sequential 적용', posType);
        end
    end
end

function comp_len = extractComponentLength(componentNode)
    % 부품 길이 추출
    comp_len = 0;
    lenNode = componentNode.getElementsByTagName('length').item(0);
    if ~isempty(lenNode)
        comp_len = str2double(lenNode.getTextContent);
    end
end

function mass = extractComponentMass(componentNode)
    % 부품 질량 추출 (명시적 질량 또는 부피와 밀도로 계산)
    mass = 0;
    
    % 직접 질량이 명시된 경우
    massNode = componentNode.getElementsByTagName('mass').item(0);
    if ~isempty(massNode)
        mass = str2double(massNode.getTextContent);
        return;
    end
    
    % 질량 부품인 경우
    if strcmp(char(componentNode.getNodeName), 'masscomponent')
        mass = str2double(getNodeValue(componentNode, 'mass', '0'));
        return;
    end
    
    % 부피와 밀도로 계산
    comp_len = extractComponentLength(componentNode);
    radius = extractComponentRadius(componentNode);
    thickness = extractComponentThickness(componentNode);
    
    % 재질 밀도 추출
    density = extractMaterialDensity(componentNode);
    
    % 형상에 따른 부피 계산
    nodeName = char(componentNode.getNodeName);
    volume = 0;
    
    switch nodeName
        case 'nosecone'
            % 노즈콘 부피 (원뿔 부피의 1/3 근사값)
            volume = pi * radius^2 * comp_len / 3;
        case 'bodytube'
            % 바디튜브 부피 (원통 - 내부 원통)
            if thickness > 0
                innerRadius = max(0, radius - thickness);
                volume = pi * comp_len * (radius^2 - innerRadius^2);
            else
                volume = pi * radius^2 * comp_len;
            end
        case 'transition'
            % 전이부 부피 (간소화된 계산)
            foreRadiusNode = componentNode.getElementsByTagName('foreradius').item(0);
            aftRadiusNode = componentNode.getElementsByTagName('aftradius').item(0);
            if ~isempty(foreRadiusNode) && ~isempty(aftRadiusNode)
                foreRadius = str2double(foreRadiusNode.getTextContent);
                aftRadius = str2double(aftRadiusNode.getTextContent);
                avgRadius = (foreRadius + aftRadius) / 2;
                volume = pi * avgRadius^2 * comp_len;
            else
                volume = pi * radius^2 * comp_len;
            end
        case 'trapezoidfinset'
            % 핀 부피
            rootChord = str2double(getNodeValue(componentNode, 'rootchord', '0'));
            tipChord = str2double(getNodeValue(componentNode, 'tipchord', '0'));
            height = str2double(getNodeValue(componentNode, 'height', '0'));
            finCount = str2double(getNodeValue(componentNode, 'fincount', '4'));
            
            singleFinArea = (rootChord + tipChord) * height / 2;
            volume = singleFinArea * thickness * finCount;
        otherwise
            volume = pi * radius^2 * comp_len;
    end
    
    % 질량 = 부피 * 밀도
    mass = volume * density;
end

function radius = extractComponentRadius(componentNode)
    % 부품 반경 추출
    radius = 0;
    radiusNode = componentNode.getElementsByTagName('radius').item(0);
    
    if ~isempty(radiusNode)
        radiusStr = char(radiusNode.getTextContent);
        if startsWith(radiusStr, 'auto')
            parts = strsplit(radiusStr);
            if numel(parts) >= 2
                try 
                    radius = str2double(parts{2}); 
                catch
                    radius = 0; 
                end
            end
        else
            try 
                radius = str2double(radiusStr); 
            catch
                radius = 0; 
            end
        end
    end
    
    % 노즈콘 aftradius 확인
    if radius == 0 && strcmp(char(componentNode.getNodeName), 'nosecone')
        aftRadiusNode = componentNode.getElementsByTagName('aftradius').item(0);
        if ~isempty(aftRadiusNode)
            radiusStr = char(aftRadiusNode.getTextContent);
            if startsWith(radiusStr, 'auto')
                parts = strsplit(radiusStr);
                if numel(parts) >= 2
                    try 
                        radius = str2double(parts{2}); 
                    catch
                        radius = 0; 
                    end
                end
            else
                try 
                    radius = str2double(radiusStr);
                catch
                    radius = 0; 
                end
            end
        end
    end
end

function thickness = extractComponentThickness(componentNode)
    % 부품 두께 추출
    thickness = 0;
    thicknessNode = componentNode.getElementsByTagName('thickness').item(0);
    if ~isempty(thicknessNode)
        thickness = str2double(thicknessNode.getTextContent);
    end
end

function density = extractMaterialDensity(componentNode)
    % 재질 밀도 추출
    density = 1000; % 기본값 (kg/m^3)
    materialNode = componentNode.getElementsByTagName('material').item(0);
    if ~isempty(materialNode)
        densityAttr = materialNode.getAttribute('density');
        if ~isempty(densityAttr)
            density = str2double(densityAttr);
        end
    end
end

function motorMass = extractMotorInfo(xmldata)
    % 모터 정보 추출
    motorMass = 9.0; % 기본값 - AeroTech M2400T 모터 무게 (대략 9kg)
    
    motorNodes = xmldata.getElementsByTagName('motor');
    if motorNodes.getLength > 0
        motorNode = motorNodes.item(0);
        manufacturer = getNodeValue(motorNode, 'manufacturer', 'Unknown');
        designation = getNodeValue(motorNode, 'designation', 'Unknown');
        
        fprintf('모터 정보 발견: %s %s\n', manufacturer, designation);
        
        % AeroTech M2400T 모터인지 확인
        if strcmp(manufacturer, 'AeroTech') && strcmp(designation, 'M2400T')
            motorMass = 9.0; % 실제 무게값 사용
        end
    end
end