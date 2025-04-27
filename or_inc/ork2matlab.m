function ork2matlab()
    
    xmlFilePath = 'rocket.xml';
    % --------------------------------------------------
    
    drawRocketProfileFromXML_V2(xmlFilePath);
    
    % =========================================================================
    % Main Function
    % =========================================================================
    function drawRocketProfileFromXML_V2(xmlFilePath)
        fprintf('OpenRocket 2D Profile Plotter V2.3 \n');
        fprintf('============================================================\n');
    
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
    
        % --- 3. 플로팅 준비 ---
        figure('Name', sprintf('Rocket Profile: %s', rocketName), 'NumberTitle', 'off');
        hold on;
        axis equal;
        grid on;
        xlabel('축 방향 위치 (m)');
        ylabel('반경 (m)');
        title(sprintf('로켓 2D 프로파일: %s', rocketName));
    
        % --- 4. 부품 처리 로직 ---
        current_absolute_end_x = 0;
        current_radius = 0;
    
        try
            stageSubcomponentsNode = findMainComponentListNode(xmldata);
            if isempty(stageSubcomponentsNode)
                error('메인 부품 목록 (<rocket> -> <subcomponents> -> <stage> -> <subcomponents>)을 찾을 수 없습니다.');
            end
    
            fprintf('\n--- 부품 처리 시작 ---\n');
            childNodes = stageSubcomponentsNode.getChildNodes;
            fprintf('메인 <subcomponents> 내에 처리할 노드 수: %d개\n', childNodes.getLength);
    
            componentAbsoluteStartX = zeros(1, childNodes.getLength);
            componentAbsoluteEndX = zeros(1, childNodes.getLength);
            componentRadius = zeros(1, childNodes.getLength);
            node_idx = 0;
    
            for i = 0:childNodes.getLength-1
                node = childNodes.item(i);
                if node.getNodeType == org.w3c.dom.Node.ELEMENT_NODE
                    node_idx = node_idx + 1;
                    nodeName = char(node.getNodeName);
                    compName = getNodeValue(node, 'name', ['Unnamed ' nodeName]);
                    fprintf('\n>> [%d] 처리 시작: %s (%s)\n', node_idx, compName, nodeName);
    
                    % --- 4.1. 부품의 절대 시작 위치 결정 ---
                    [absolute_start_x, position_type_desc] = getAbsoluteStartPosition(node, current_absolute_end_x);
                    fprintf('    절대 시작 위치 결정: %.4f m (기준: %s)\n', absolute_start_x, position_type_desc);
                    componentAbsoluteStartX(node_idx) = absolute_start_x;
                    current_x = absolute_start_x;
    
                    % --- 4.2. 부품 타입별 처리 ---
                    component_len = 0;
                    component_end_radius = current_radius;
    
                    switch nodeName
                        case 'nosecone'
                            lengthNode = node.getElementsByTagName('length').item(0);
                            aftRadiusNode = node.getElementsByTagName('aftradius').item(0);
                            shapeNode = node.getElementsByTagName('shape').item(0);
                            shapeParamNode = node.getElementsByTagName('shapeparameter').item(0);
    
                            if ~isempty(lengthNode) && ~isempty(aftRadiusNode) && ~isempty(shapeNode)
                                len = str2double(lengthNode.getTextContent);
                                aftRadiusStr = char(aftRadiusNode.getTextContent);
                                shapeType = char(shapeNode.getTextContent);
                                shapeParam = 0;
                                if ~isempty(shapeParamNode), shapeParam = str2double(shapeParamNode.getTextContent); end
                                aftRadius = parseRadius(aftRadiusStr, 0.068);
    
                                fprintf('    형태: %s, 길이: %.3f, 끝 반경: %.3f\n', shapeType, len, aftRadius);
                                plotNoseConeShape(shapeType, shapeParam, len, aftRadius, current_x, 'b');
                                component_len = len;
                                component_end_radius = aftRadius;
                            else
                                fprintf('    경고: Nose Cone 정보 부족. 건너뜀.\n');
                            end
    
                        case 'bodytube'
                            lengthNode = node.getElementsByTagName('length').item(0);
                            radiusNode = node.getElementsByTagName('radius').item(0);
    
                            if ~isempty(lengthNode) && ~isempty(radiusNode)
                                len = str2double(lengthNode.getTextContent);
                                radiusStr = char(radiusNode.getTextContent);
                                tube_radius = parseRadius(radiusStr, current_radius);
    
                                fprintf('    길이: %.3f, 반경: %.3f\n', len, tube_radius);
                                if abs(tube_radius - current_radius) > 1e-6 && current_radius ~= 0
                                    fprintf('      주의: 반경 불일치 감지 (%.4f -> %.4f).\n', current_radius, tube_radius);
                                end
    
                                x_start = current_x;
                                x_end = current_x + len;
                                plot([x_start, x_end, x_end, x_start, x_start], ...
                                     [tube_radius, tube_radius, -tube_radius, -tube_radius, tube_radius], 'k-', 'LineWidth', 1.5);
                                component_len = len;
                                component_end_radius = tube_radius;
    
                                % Fin Set 처리 (Body Tube 내부)
                                finSetNodes = getChildElementsByTagName(node, 'trapezoidfinset');
                                fprintf('    내부 Fin Set 검색 결과: %d개\n', finSetNodes.getLength);
                                for j = 0:finSetNodes.getLength-1
                                    % *** 수정된 processFinSet 함수 호출 ***
                                    processFinSet(finSetNodes.item(j), x_start, len, tube_radius);
                                end
                            else
                                fprintf('    경고: Body Tube 정보 부족. 건너뜀.\n');
                            end
    
                        case 'transition'
                            lengthNode = node.getElementsByTagName('length').item(0);
                            foreRadiusNode = node.getElementsByTagName('foreradius').item(0);
                            aftRadiusNode = node.getElementsByTagName('aftradius').item(0);
                            shapeNode = node.getElementsByTagName('shape').item(0);
    
                            if ~isempty(lengthNode) && ~isempty(foreRadiusNode) && ~isempty(aftRadiusNode) && ~isempty(shapeNode)
                                len = str2double(lengthNode.getTextContent);
                                foreRadiusStr = char(foreRadiusNode.getTextContent);
                                aftRadiusStr = char(aftRadiusNode.getTextContent);
                                shapeType = char(shapeNode.getTextContent);
    
                                foreRadius = parseRadius(foreRadiusStr, current_radius);
                                aftRadius = parseRadius(aftRadiusStr, foreRadius * 0.8);
    
                                fprintf('    형태: %s, 길이: %.3f, Fore반경: %.3f, Aft반경: %.3f\n', shapeType, len, foreRadius, aftRadius);
    
                                if abs(foreRadius - current_radius) > 1e-6 && current_radius ~= 0
                                    fprintf('      주의: Transition 시작 반경 불일치 감지 (이전: %.4f, Fore: %.4f).\n', current_radius, foreRadius);
                                end
                                start_radius_actual = current_radius;
                                fprintf('      실제 시작 반경: %.4f\n', start_radius_actual);
    
                                x_start = current_x;
                                x_end = current_x + len;
                                if strcmp(shapeType, 'conical')
                                    plot([x_start, x_end, x_end, x_start, x_start], ...
                                         [start_radius_actual, aftRadius, -aftRadius, -start_radius_actual, start_radius_actual], 'm-', 'LineWidth', 1.5);
                                else
                                    fprintf('      경고: Transition 형태 "%s"는 Conical로 근사.\n', shapeType);
                                     plot([x_start, x_end, x_end, x_start, x_start], ...
                                         [start_radius_actual, aftRadius, -aftRadius, -start_radius_actual, start_radius_actual], 'm:', 'LineWidth', 1.5);
                                end
                                component_len = len;
                                component_end_radius = aftRadius;
                            else
                                 fprintf('    경고: Transition 정보 부족. 건너뜀.\n');
                            end
    
                        otherwise
                            fprintf('    지원되지 않는 주요 부품 타입: %s. 위치만 계산.\n', nodeName);
                            lenNode = node.getElementsByTagName('length').item(0);
                            if ~isempty(lenNode)
                                component_len = str2double(lenNode.getTextContent());
                                 fprintf('      길이 정보(%.3f)는 위치 계산에 반영.\n', component_len);
                            else component_len = 0; end
                            component_end_radius = current_radius;
                    end
    
                    % --- 4.3. 현재 부품의 끝 위치 및 다음 부품 시작 위치 업데이트 ---
                    absolute_end_x = absolute_start_x + component_len;
                    componentAbsoluteEndX(node_idx) = absolute_end_x;
                    componentRadius(node_idx) = component_end_radius;
                    current_absolute_end_x = absolute_end_x;
                    current_radius = component_end_radius;
                    fprintf('    처리 완료. 절대 끝 위치: %.4f m, 끝 반경: %.4f m\n', absolute_end_x, current_radius);
    
                end % if Element Node
            end % for childNodes loop
    
        catch ME
            fprintf(2, '\n*** 처리 중 심각한 오류 발생 ***\n');
            fprintf(2, '오류 메시지: %s\n', ME.message);
            fprintf(2, '오류 위치: %s, %d번째 줄\n', ME.stack(1).name, ME.stack(1).line);
            if ~isempty(ME.cause), fprintf(2, '원인: %s\n', ME.cause{1}.message); end
        end
    
        % --- 5. 플롯 마무리 ---
        hold off; axis tight;
        current_ylim = ylim;
        if ~isempty(current_ylim) && diff(current_ylim) > 1e-6
            ylim_range = current_ylim(2) - current_ylim(1);
            ylim([current_ylim(1) - 0.1*ylim_range, current_ylim(2) + 0.1*ylim_range]);
        end
        grid on; title(sprintf('로켓 2D 프로파일: %s ', rocketName)); % 제목 수정
    
        if node_idx == 0
            fprintf('\n경고: 유효한 로켓 부품이 XML에서 발견/처리되지 못했습니다.\n');
        else
            total_length = max(componentAbsoluteEndX(1:node_idx));
            fprintf('\n--- 최종 결과 ---\n');
            fprintf('총 계산된 길이: %.4f m\n', total_length);
            fprintf('2D 프로파일 그리기가 완료되었습니다.\n');
        end
         fprintf('참고: 모든 부품 타입, 복잡한 내부 구조, 3D 형상은 완벽히 반영되지 않을 수 있습니다.\n');
         fprintf('      경고: 모든 핀 위치가 TE(뒷전) 기준으로 계산되었습니다. 카나드 등 일부 핀 위치가 어색할 수 있습니다.\n'); % 경고 추가
    
    end
    
    % =========================================================================
    % Helper Functions
    % =========================================================================
    
    % --- XML 정보 추출 함수 ---
    function [rocketName, designer] = getRocketInfo(xmldata)
        rocketName = 'Unknown Rocket'; designer = 'Unknown';
        rocketNode = xmldata.getElementsByTagName('rocket').item(0);
        if isempty(rocketNode), return; end
        rocketName = getNodeValue(rocketNode, 'name', 'Name Not Found');
        designer = getNodeValue(rocketNode, 'designer', 'Unknown');
    end
    
    function nodeValue = getNodeValue(parentNode, tagName, defaultValue)
        nodeList = parentNode.getElementsByTagName(tagName);
        if nodeList.getLength > 0 && ~isempty(nodeList.item(0).getFirstChild)
            nodeValue = char(nodeList.item(0).getTextContent);
        else nodeValue = defaultValue; end
    end
    
    function subCompNode = findMainComponentListNode(xmldata)
        subCompNode = [];
        rocketNode = xmldata.getElementsByTagName('rocket').item(0);
        if isempty(rocketNode), fprintf('오류: <rocket> 없음.\n'); return; end
        rocketSubNode = findDirectChildElement(rocketNode, 'subcomponents');
        if isempty(rocketSubNode), fprintf('오류: <rocket> 직속 <subcomponents> 없음.\n'); return; end
        mainStageNode = findDirectChildElement(rocketSubNode, 'stage');
         if isempty(mainStageNode), fprintf('오류: <rocket>/<subcomponents> 직속 <stage> 없음.\n'); return; end
        subCompNode = findDirectChildElement(mainStageNode, 'subcomponents');
         if isempty(subCompNode), fprintf('오류: <stage> 직속 <subcomponents> (부품 목록) 없음.\n'); return; end
        fprintf('메인 부품 목록 노드 찾기 성공.\n');
    end
    
    function childElement = findDirectChildElement(parentNode, tagName)
        childElement = []; childNodes = parentNode.getChildNodes;
        for k = 0:childNodes.getLength-1
            child = childNodes.item(k);
            if child.getNodeType == org.w3c.dom.Node.ELEMENT_NODE && strcmp(char(child.getNodeName), tagName)
                childElement = child; return;
            end
        end
    end
    
    function childElements = getChildElementsByTagName(parentNode, tagName)
        childElements = parentNode.getElementsByTagName(tagName);
    end
    
    % --- 위치 계산 함수 ---
    function [start_x, position_type_desc] = getAbsoluteStartPosition(componentNode, default_start_x)
        start_x = default_start_x; position_type_desc = 'Sequential';
        posList = getChildElementsByTagName(componentNode, 'position');
        if posList.getLength > 0
            posNode = posList.item(0); posValue = str2double(posNode.getTextContent());
            posType = char(posNode.getAttribute('type'));
            if strcmp(posType, 'absolute')
                start_x = posValue; position_type_desc = sprintf('Absolute (%.4f)', posValue);
            else position_type_desc = sprintf('Relative type "%s" - Sequential 적용', posType); end
        end
    end
    
    % --- 반경 처리 함수 ---
    function radius = parseRadius(radiusStr, default_radius)
        if isempty(radiusStr), fprintf('    경고: 반경 문자열 비어있음, 기본값(%.4f) 사용.\n', default_radius); radius = default_radius; return; end
        if startsWith(radiusStr, 'auto')
            parts = strsplit(radiusStr);
            if length(parts) >= 2
                try radius = str2double(parts{2}); catch, fprintf('    경고: ''auto'' 반경 파싱 오류 ("%s"), 기본값(%.4f) 사용.\n', radiusStr, default_radius); radius = default_radius; end
            else radius = default_radius; end
        else
            try radius = str2double(radiusStr); catch, fprintf('    경고: 반경 파싱 오류 ("%s"), 기본값(%.4f) 사용.\n', radiusStr, default_radius); radius = default_radius; end
        end
        if radius < 0 || radius > 10 || isnan(radius), fprintf('    경고: 비정상/NaN 반경 값(%.4f) 감지, 기본값(%.4f) 사용.\n', radius, default_radius); radius = default_radius; end
    end
    
    % --- 형상 그리기 함수 ---
    function plotNoseConeShape(shapeType, shapeParam, L, R, start_x, color)
        n_points = 100; x_local = linspace(0, L, n_points); y_local = zeros(1, n_points);
        fprintf('      노즈콘 그리기: 형태=%s, 파라미터=%.2f\n', shapeType, shapeParam);
        try
            if L <= 0 || R <= 0, error('Nosecone 길이/반경 0 이하'); end
            switch lower(shapeType)
                case 'ogive', rho = (R^2 + L^2) / (2*R); y_local = sqrt(max(0, rho^2 - (L - x_local).^2)) + R - rho;
                case 'conical', y_local = R * (x_local / L);
                case 'elliptical', y_local = R * sqrt(max(0, 1 - ((x_local-L)/L).^2));
                case 'power', n_pow = shapeParam; if n_pow <= 0 || n_pow > 1, n_pow = 0.5; fprintf(' 경고: Power 지수 범위 오류, n=0.5 가정.'); end; y_local = R * (x_local / L).^n_pow;
                case 'haack', theta = acos(max(-1, min(1, 1 - 2*x_local/L))); y_local = (R / sqrt(pi)) * sqrt(max(0, theta - 0.5*sin(2*theta)));
                otherwise, fprintf('      경고: 미지원 형태 "%s". Conical로 근사.\n', shapeType); y_local = R * (x_local / L); color = [color, ':'];
            end
            y_local(1) = 0; y_local(end) = R; y_local(isnan(y_local) | ~isreal(y_local)) = 0; y_local = max(0, y_local);
        catch ME, fprintf('      오류: 노즈콘 계산 오류. Conical 대체. (%s)\n', ME.message); y_local = R * (x_local / L); color = [color, ':']; y_local(1) = 0; y_local(end) = R; end
        x_abs = start_x + x_local;
        plot(x_abs, y_local, '-', 'Color', color, 'LineWidth', 1.5); plot(x_abs, -y_local, '-', 'Color', color, 'LineWidth', 1.5);
        plot([start_x + L, start_x + L], [-R, R], '-', 'Color', color, 'LineWidth', 1.5);
    end
    
    % --- 핀 처리 함수 ---
    function processFinSet(finSetNode, parent_abs_start_x, parent_len, parent_radius)
        finName = getNodeValue(finSetNode, 'name', 'Unnamed Fins');
        fprintf('      Fin Set 처리 중: "%s"\n', finName);
        fprintf('        부모 Tube 정보: 시작=%.4f, 끝=%.4f, 길이=%.4f, 반경=%.4f\n', ...
                parent_abs_start_x, parent_abs_start_x + parent_len, parent_len, parent_radius);
    
        % 핀 치수 읽기
        rootChord = str2double(getNodeValue(finSetNode, 'rootchord', '0'));
        tipChord = str2double(getNodeValue(finSetNode, 'tipchord', '0'));
        finHeight = str2double(getNodeValue(finSetNode, 'height', '0'));
        sweepLength = str2double(getNodeValue(finSetNode, 'sweeplength', '0'));
        thickness = str2double(getNodeValue(finSetNode, 'thickness', '0.001'));
    
        if rootChord <= 0 || finHeight <= 0
            fprintf('        경고: Fin Set "%s" 치수 오류. 그릴 수 없음.\n', finName);
            return;
        end
    
        % --- 위치 계산 (모든 핀을 TE 기준으로 계산) ---
        ref_coord_x = parent_abs_start_x; % 기본값
        position_type_desc = 'Default (Parent Top)';
    
        posList = getChildElementsByTagName(finSetNode, 'position');
        if posList.getLength > 0
            posNode = posList.item(0);
            finPosVal = str2double(posNode.getTextContent());
            finPosType = char(posNode.getAttribute('type'));
            parent_abs_end_x = parent_abs_start_x + parent_len;
            parent_abs_mid_x = parent_abs_start_x + parent_len / 2;
    
            % ref_coord_x 계산 (Trailing Edge Root의 X좌표)
            switch finPosType
                case 'bottom'
                     % + 값은 우측(aft) 이동으로 해석
                     ref_coord_x = parent_abs_end_x + finPosVal; % *** 계산식: + 사용 ***
                     position_type_desc = sprintf('Relative Bottom (Offset %.4f from end)', finPosVal);
                     fprintf('        위치 해석 (Bottom, TE 기준): 부모 끝(%.3f) + offset(%.3f) = %.3f\n', parent_abs_end_x, finPosVal, ref_coord_x);
                case 'top'
                     % + 값은 우측(aft) 이동으로 해석
                     ref_coord_x = parent_abs_start_x + finPosVal;
                     position_type_desc = sprintf('Relative Top (Offset %.4f)', finPosVal);
                     fprintf('        위치 해석 (Top, TE 기준): 부모 시작(%.3f) + offset(%.3f) = %.3f\n', parent_abs_start_x, finPosVal, ref_coord_x);
                case 'middle'
                     % + 값은 우측(aft) 이동으로 해석
                     ref_coord_x = parent_abs_mid_x + finPosVal;
                     position_type_desc = sprintf('Relative Middle (Offset %.4f)', finPosVal);
                     fprintf('        위치 해석 (Middle, TE 기준): 부모 중간(%.3f) + offset(%.3f) = %.3f\n', parent_abs_mid_x, finPosVal, ref_coord_x);
                case 'absolute'
                     ref_coord_x = finPosVal; % 절대 위치
                     position_type_desc = sprintf('Absolute (%.4f)', finPosVal);
                     fprintf('        위치 해석 (Absolute, TE 기준): %.3f\n', ref_coord_x);
                otherwise
                     % 알 수 없는 타입도 일단 '+' 오프셋 적용
                     ref_coord_x = parent_abs_start_x + finPosVal;
                     position_type_desc = sprintf('Unknown Type "%s" - Parent Top 적용', finPosType);
                     fprintf('        위치 해석 (Unknown, TE 기준): 부모 시작(%.3f) + offset(%.3f) = %.3f\n', parent_abs_start_x, finPosVal, ref_coord_x);
            end
        else
             % position 태그 없으면 TE를 부모 시작점에 배치 (매우 부자연스러움)
             ref_coord_x = parent_abs_start_x;
             position_type_desc = 'Default (Parent Top - TE 기준)';
             fprintf('        경고: <position> 태그 없음. TE를 부모 시작점에 배치.\n', finName);
        end
    
        % --- 핀 꼭지점 좌표 계산 (항상 TE 기준) ---
        x2 = ref_coord_x;                           % Trailing Edge Root X (계산된 위치)
        x1 = x2 - rootChord;                        % Leading Edge Root X 계산
        x4 = x1 + sweepLength;                      % Leading Edge Tip X
        x3 = x4 + tipChord;                         % Trailing Edge Tip X
        y1 = parent_radius; y2 = parent_radius;     % Root Y
        y3 = parent_radius + finHeight; y4 = parent_radius + finHeight; % Tip Y
    
        fprintf('        ==> 계산된 핀 Trailing Edge Root 축 위치 (x2): %.4f m (기준: %s)\n', x2, position_type_desc);
        fprintf('        ==> 계산된 핀 Leading Edge Root 축 위치 (x1): %.4f m\n', x1);
        fprintf('        핀 치수: Root=%.3f, Tip=%.3f, Height=%.3f, Sweep=%.3f, Thickness=%.3f\n', rootChord, tipChord, finHeight, sweepLength, thickness);
    
        fin_x = [x1, x2, x3, x4, x1];
        fin_y_top = [y1, y2, y3, y4, y1];
        fin_y_bottom = [-y1, -y2, -y3, -y4, -y1];
    
        plot(fin_x, fin_y_top, 'r-', 'LineWidth', 1);
        plot(fin_x, fin_y_bottom, 'r-', 'LineWidth', 1);
    end
    
    % --- Ternary 연산자 헬퍼 ---
    % (fprintf에서 조건부 문자열 사용을 위해 유지, 현재 processFinSet에서는 직접 사용 안 함)
    function result = ternary(condition, true_val, false_val)
        if condition
            result = true_val;
        else
            result = false_val;
        end
    end

end