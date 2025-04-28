function ork2matlab3d()
   
    xmlFilePath = 'rocket.xml';
    n_theta_points = 30; % 3D 회전 시 원주 방향 점 개수 (높을수록 부드러움)
    
    drawRocketProfile3D(xmlFilePath, n_theta_points);
    
    % =========================================================================
    % Main Function
    % =========================================================================
    function drawRocketProfile3D(xmlFilePath, n_theta)
        fprintf('OpenRocket 3D Profile Plotter V3.0 (Experimental)\n');
        fprintf('===================================================\n');
    
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
    
        % --- 3. 3D 플로팅 준비 ---
        figure('Name', sprintf('Rocket 3D Model: %s', rocketName), 'NumberTitle', 'off');
        hold on;
        axis equal; % 중요: 가로 세로 비율 유지
        grid on;
        xlabel('X (Axial, m)');
        ylabel('Y (m)');
        zlabel('Z (m)');
        title(sprintf('로켓 3D 모델: %s', rocketName));
        view(3); % 3D 뷰 설정 (azimuth = -37.5, elevation = 30)
        rotate3d on; % 마우스로 회전 가능하게
          
        % --- 4. 부품 처리 로직 ---
        current_absolute_end_x = 0;
        current_radius = 0;
        rocket_max_length = 0;  % 로켓 최대 길이 변수 추가 (화살표 크기 계산용)
        rocket_max_radius = 0;  % 로켓 최대 반경 변수 추가 (화살표 크기 계산용)
    
        try
            stageSubcomponentsNode = findMainComponentListNode(xmldata);
            if isempty(stageSubcomponentsNode)
                error('메인 부품 목록을 찾을 수 없습니다.');
            end
    
            fprintf('\n--- 부품 처리 시작 (3D) ---\n');
            childNodes = stageSubcomponentsNode.getChildNodes;
            fprintf('메인 <subcomponents> 내에 처리할 노드 수: %d개\n', childNodes.getLength);
    
            node_idx = 0;
    
            for i = 0:childNodes.getLength-1
                node = childNodes.item(i);
                if node.getNodeType == org.w3c.dom.Node.ELEMENT_NODE
                    node_idx = node_idx + 1;
                    nodeName = char(node.getNodeName);
                    compName = getNodeValue(node, 'name', ['Unnamed ' nodeName]);
                    fprintf('\n>> [%d] 처리 시작: %s (%s)\n', node_idx, compName, nodeName);
    
                    [absolute_start_x, position_type_desc] = getAbsoluteStartPosition(node, current_absolute_end_x);
                    fprintf('    절대 시작 위치 결정: %.4f m (기준: %s)\n', absolute_start_x, position_type_desc);
                    current_x = absolute_start_x;
    
                    component_len = 0;
                    component_end_radius = current_radius;
                    component_color = 'cyan'; % 기본 색상
    
                    switch nodeName
                        case 'nosecone'
                            component_color = 'blue';
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
                                plotNoseCone3D(shapeType, shapeParam, len, aftRadius, current_x, component_color, n_theta);
                                component_len = len;
                                component_end_radius = aftRadius;
                                rocket_max_radius = max(rocket_max_radius, aftRadius); % 최대 반경 업데이트
                            else fprintf('    경고: Nose Cone 정보 부족.\n'); end
    
                        case 'bodytube'
                            component_color = [0.3 0.3 0.3]; % Dark Gray
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
    
                                plotBodyTube3D(current_x, len, tube_radius, component_color, n_theta);
                                component_len = len;
                                component_end_radius = tube_radius;
                                rocket_max_radius = max(rocket_max_radius, tube_radius); % 최대 반경 업데이트
    
                                % Fin Set 처리
                                finSetNodes = getChildElementsByTagName(node, 'trapezoidfinset');
                                fprintf('    내부 Fin Set 검색 결과: %d개\n', finSetNodes.getLength);
                                finCountNode = findDirectChildElement(node, 'fincount'); % 바디튜브 직접 자식 확인 (가정)
                                finCount = 4; % 기본값
                                if ~isempty(finCountNode), finCount = str2double(finCountNode.getTextContent); end
    
                                for j = 0:finSetNodes.getLength-1
                                    % *** 3D 핀 그리기 함수 호출 ***
                                    [fin_max_height] = processFinSet3D(finSetNodes.item(j), current_x, len, tube_radius, finCount, n_theta);
                                    % 핀 높이를 고려한 최대 반경 업데이트
                                    rocket_max_radius = max(rocket_max_radius, tube_radius + fin_max_height);
                                end
                            else fprintf('    경고: Body Tube 정보 부족.\n'); end
    
                        case 'transition'
                            component_color = 'magenta';
                            lengthNode = node.getElementsByTagName('length').item(0);
                            foreRadiusNode = node.getElementsByTagName('foreradius').item(0);
                            aftRadiusNode = node.getElementsByTagName('aftradius').item(0);
                            shapeNode = node.getElementsByTagName('shape').item(0);
    
                            if ~isempty(lengthNode) && ~isempty(foreRadiusNode) && ~isempty(aftRadiusNode) && ~isempty(shapeNode)
                                len = str2double(lengthNode.getTextContent);
                                foreRadiusStr = char(foreRadiusNode.getTextContent);
                                aftRadiusStr = char(aftRadiusNode.getTextContent);
                                shapeType = char(shapeNode.getTextContent); % 보통 'conical'
    
                                foreRadius = parseRadius(foreRadiusStr, current_radius);
                                aftRadius = parseRadius(aftRadiusStr, foreRadius * 0.8);
    
                                 fprintf('    형태: %s, 길이: %.3f, Fore반경: %.3f, Aft반경: %.3f\n', shapeType, len, foreRadius, aftRadius);
                                if abs(foreRadius - current_radius) > 1e-6 && current_radius ~= 0
                                    fprintf('      주의: 반경 불일치 (%.4f -> %.4f).\n', current_radius, foreRadius);
                                end
                                start_radius_actual = current_radius;
    
                                plotTransition3D(current_x, len, start_radius_actual, aftRadius, component_color, n_theta);
                                component_len = len;
                                component_end_radius = aftRadius;
                                rocket_max_radius = max(rocket_max_radius, max(foreRadius, aftRadius)); % 최대 반경 업데이트
                            else fprintf('    경고: Transition 정보 부족.\n'); end
    
                        otherwise
                            fprintf('    지원되지 않는 주요 부품 타입: %s. 3D 형상 없음.\n', nodeName);
                            lenNode = node.getElementsByTagName('length').item(0);
                            if ~isempty(lenNode)
                                component_len = str2double(lenNode.getTextContent());
                                 fprintf('      길이 정보(%.3f)는 위치 계산에 반영.\n', component_len);
                            else component_len = 0; end
                            component_end_radius = current_radius;
                    end
    
                    absolute_end_x = absolute_start_x + component_len;
                    current_absolute_end_x = absolute_end_x;
                    current_radius = component_end_radius;
                    fprintf('    처리 완료. 절대 끝 위치: %.4f m, 끝 반경: %.4f m\n', absolute_end_x, current_radius);
    
                    % 로켓 최대 길이 업데이트
                    rocket_max_length = max(rocket_max_length, absolute_end_x);
    
                end % if Element Node
            end % for childNodes loop
    
            % --- 좌표축 그리기 ---
            % 로켓 크기 기반으로 적절한 화살표 크기 계산
            arrow_length = max(rocket_max_length, rocket_max_radius * 5) * 1.15; % 로켓보다 약간 큰 정도
            arrow_width = arrow_length * 0.025; % 화살표 두께는 길이의 2.5%
            
            % X축 (빨간색)
            quiver3(0, 0, 0, arrow_length, 0, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.1);
            text(arrow_length*1.05, 0, 0, 'X', 'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');
            
            % Y축 (녹색)
            quiver3(0, 0, 0, 0, rocket_max_radius* 1.15, 0, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.8);
            text(0, rocket_max_radius*1.55, 0, 'Y', 'Color', 'g', 'FontSize', 12, 'FontWeight', 'bold');
            
            % Z축 (파란색)
            quiver3(0, 0, 0, 0, 0, rocket_max_radius* 1.15, 0, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.8);
            text(0, 0, rocket_max_radius*1.55, 'Z', 'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold');
            
            fprintf('\n좌표축을 (0,0,0) 위치에 추가했습니다 (X:빨강, Y:녹색, Z:파랑).\n');
    
        catch ME
            fprintf(2, '\n*** 처리 중 심각한 오류 발생 ***\n');
            fprintf(2, '오류 메시지: %s\n', ME.message);
            fprintf(2, '오류 위치: %s, %d번째 줄\n', ME.stack(1).name, ME.stack(1).line);
            if ~isempty(ME.cause), fprintf(2, '원인: %s\n', ME.cause{1}.message); end
        end
    
        % --- 5. 플롯 마무리 ---
        hold off;
        axis tight; % 축 범위 자동 조절 시도
        daspect([1 1 1]); % 데이터 단위 비율 1:1:1 (형태 왜곡 방지)
        camlight left; % 좌측에서 조명 비추기
        lighting gouraud; % 부드러운 조명 효과
        material dull; % 재질 설정 (반사 줄임)
        fprintf('\n3D 모델 그리기가 완료되었습니다 (실험적).\n');
        fprintf('참고: 핀 두께/형상, 내부 부품 등은 단순화/생략되었습니다.\n');
    
    end
    
    % =========================================================================
    % Helper Functions (기존 함수 + 3D 그리기 함수 추가)
    % =========================================================================
    
    % --- XML 정보 추출 (기존과 동일) ---
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
        subCompNode = []; rocketNode = xmldata.getElementsByTagName('rocket').item(0); if isempty(rocketNode), return; end
        rocketSubNode = findDirectChildElement(rocketNode, 'subcomponents'); if isempty(rocketSubNode), return; end
        mainStageNode = findDirectChildElement(rocketSubNode, 'stage'); if isempty(mainStageNode), return; end
        subCompNode = findDirectChildElement(mainStageNode, 'subcomponents'); if isempty(subCompNode), fprintf('오류: <stage> 직속 <subcomponents> 없음.\n'); end
    end
    function childElement = findDirectChildElement(parentNode, tagName)
        childElement = []; childNodes = parentNode.getChildNodes;
        for k = 0:childNodes.getLength-1, child = childNodes.item(k); if child.getNodeType == org.w3c.dom.Node.ELEMENT_NODE && strcmp(char(child.getNodeName), tagName), childElement = child; return; end, end
    end
    function childElements = getChildElementsByTagName(parentNode, tagName)
        childElements = parentNode.getElementsByTagName(tagName);
    end
    function [start_x, position_type_desc] = getAbsoluteStartPosition(componentNode, default_start_x)
        start_x = default_start_x; position_type_desc = 'Sequential'; posList = getChildElementsByTagName(componentNode, 'position');
        if posList.getLength > 0, posNode = posList.item(0); posValue = str2double(posNode.getTextContent()); posType = char(posNode.getAttribute('type')); if strcmp(posType, 'absolute'), start_x = posValue; position_type_desc = sprintf('Absolute (%.4f)', posValue); else position_type_desc = sprintf('Relative type "%s" - Seq', posType); end, end
    end
    function radius = parseRadius(radiusStr, default_radius)
        if isempty(radiusStr), radius = default_radius; return; end
        if startsWith(radiusStr, 'auto'), parts = strsplit(radiusStr); if length(parts) >= 2, try radius = str2double(parts{2}); catch, radius = default_radius; end, else radius = default_radius; end
        else try radius = str2double(radiusStr); catch, radius = default_radius; end, end
        if radius < 0 || radius > 10 || isnan(radius), radius = default_radius; end
    end
    
    % --- 3D 형상 그리기 함수 ---
    
    function plotNoseCone3D(shapeType, shapeParam, L, R, start_x, color, n_theta)
        % 다양한 노즈콘 형상을 3D Surface로 그림
        n_points = 50; % 축방향 점 개수
        x_local = linspace(0, L, n_points);
        y_local = zeros(1, n_points); % 반경
    
        fprintf('      3D 노즈콘 그리기: 형태=%s\n', shapeType);
        try
            if L <= 0 || R <= 0, error('Nosecone L/R 오류'); end
            switch lower(shapeType) % 계산 로직은 2D와 동일
                 case 'ogive', rho = (R^2 + L^2) / (2*R); y_local = sqrt(max(0, rho^2 - (L - x_local).^2)) + R - rho;
                 case 'conical', y_local = R * (x_local / L);
                 case 'elliptical', y_local = R * sqrt(max(0, 1 - ((x_local-L)/L).^2));
                 case 'power', n_pow = shapeParam; if n_pow <= 0 || n_pow > 1, n_pow = 0.5; end; y_local = R * (x_local / L).^n_pow;
                 case 'haack', theta = acos(max(-1, min(1, 1 - 2*x_local/L))); y_local = (R / sqrt(pi)) * sqrt(max(0, theta - 0.5*sin(2*theta)));
                 otherwise, fprintf(' 경고: 미지원 형태 "%s". Conical 가정.\n', shapeType); y_local = R * (x_local / L);
            end
            y_local(1) = 0; y_local(end) = R; y_local(isnan(y_local) | ~isreal(y_local)) = 0; y_local = max(0, y_local);
        catch ME, fprintf(' 오류: 노즈콘 계산 실패. (%s)\n', ME.message); return; end
    
        % 3D Surface 생성
        theta = linspace(0, 2*pi, n_theta);
        X_surf = start_x + repmat(x_local(:), 1, n_theta); % X 좌표 확장
        Y_surf = y_local(:) * cos(theta);                  % Y = r * cos(theta)
        Z_surf = y_local(:) * sin(theta);                  % Z = r * sin(theta)
    
        surf(X_surf, Y_surf, Z_surf, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 1.0);
    end
    
    function plotBodyTube3D(start_x, len, radius, color, n_theta)
        % Body Tube를 3D Cylinder로 그림 (수정된 버전)
        fprintf('      3D 바디튜브 그리기: 시작=%.3f, 길이=%.3f, 반경=%.3f\n', start_x, len, radius);
        if len <= 0 || radius <= 0
            fprintf('        경고: 길이가 0 이하이거나 반경이 0 이하인 바디튜브는 그리지 않습니다.\n');
            return;
        end
    
        % 단위 실린더 생성 (반지름=1, 높이=1, 중심축=Z축)
        [X_unit, Y_unit, Z_unit] = cylinder(1, n_theta - 1); % N각형 기반 원기둥 생성
    
        % surf 함수를 위한 좌표 스케일링 및 이동 (로켓 중심축=X축)
        X_surf = start_x + Z_unit * len;   % cylinder의 Z(높이)를 X축(길이)으로 사용, 스케일 및 이동
        Y_surf = Y_unit * radius;          % cylinder의 Y를 스케일링하여 surf의 Y로 사용
        Z_surf = X_unit * radius;          % cylinder의 X를 스케일링하여 surf의 Z로 사용
    
        % Surface 그리기
        surf(X_surf, Y_surf, Z_surf, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 1.0);
    end
    
    function plotTransition3D(start_x, len, start_radius, end_radius, color, n_theta)
        % Transition (Conical 가정)을 3D Surface로 그림
         fprintf('      3D 트랜지션 그리기: 시작=%.3f, 길이=%.3f, 시작반경=%.3f, 끝반경=%.3f\n', start_x, len, start_radius, end_radius);
        if len <= 0, return; end
    
        % 시작점과 끝점의 프로파일 정의
        x_local = [0, len];
        r_profile = [start_radius, end_radius];
    
        % 3D Surface 생성
        theta = linspace(0, 2*pi, n_theta);
        X_surf = start_x + repmat(x_local(:), 1, n_theta);
        Y_surf = r_profile(:) * cos(theta);
        Z_surf = r_profile(:) * sin(theta);
    
        surf(X_surf, Y_surf, Z_surf, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 1.0);
    end
    
    function [fin_max_height] = processFinSet3D(finSetNode, parent_abs_start_x, parent_len, parent_radius, fin_count, n_theta)
        % Fin Set을 3D Patch (단일 면)로 그림 (두께 무시)
        finName = getNodeValue(finSetNode, 'name', 'Unnamed Fins');
        fprintf('      3D Fin Set 처리 중: "%s"\n', finName);
    
        % 핀 치수 읽기
        rootChord = str2double(getNodeValue(finSetNode, 'rootchord', '0'));
        tipChord = str2double(getNodeValue(finSetNode, 'tipchord', '0'));
        finHeight = str2double(getNodeValue(finSetNode, 'height', '0'));
        sweepLength = str2double(getNodeValue(finSetNode, 'sweeplength', '0'));
        thickness = str2double(getNodeValue(finSetNode, 'thickness', '0.001')); % 두께 정보는 사용 안함
    
        if rootChord <= 0 || finHeight <= 0, fprintf(' 경고: 치수 오류.\n'); return; end
    
        % --- 위치 계산 (TE 기준 - Universal 적용) ---
        ref_coord_x = parent_abs_start_x; % 기본값
        posList = getChildElementsByTagName(finSetNode, 'position');
        if posList.getLength > 0
            posNode = posList.item(0); finPosVal = str2double(posNode.getTextContent());
            finPosType = char(posNode.getAttribute('type'));
            parent_abs_end_x = parent_abs_start_x + parent_len; parent_abs_mid_x = parent_abs_start_x + parent_len / 2;
            switch finPosType % ref_coord_x는 TE Root X
                case 'bottom', ref_coord_x = parent_abs_end_x + finPosVal;
                case 'top', ref_coord_x = parent_abs_start_x + finPosVal;
                case 'middle', ref_coord_x = parent_abs_mid_x + finPosVal;
                case 'absolute', ref_coord_x = finPosVal;
                otherwise, ref_coord_x = parent_abs_start_x + finPosVal;
            end
        else ref_coord_x = parent_abs_start_x; 
        end % 태그 없으면 TE를 부모 시작점에
    
        % --- 핀 꼭지점 좌표 계산 (항상 TE 기준) ---
        x2 = ref_coord_x;       % Trailing Edge Root X
        x1 = x2 - rootChord;    % Leading Edge Root X
        x4 = x1 + sweepLength;  % Leading Edge Tip X
        x3 = x4 + tipChord;     % Trailing Edge Tip X
        y1 = parent_radius;     y2 = parent_radius; % Root Height (== parent radius)
        y3 = parent_radius + finHeight; y4 = parent_radius + finHeight; % Tip Height
    
        fprintf('        계산된 TE Root X (x2): %.4f m, LE Root X (x1): %.4f m\n', x2, x1);
    
        % 2D 꼭지점 정의 (Y가 높이를 나타냄)
        verts_2d = [x1 y1; x2 y2; x3 y3; x4 y4];
    
        % 핀 개수만큼 회전하며 그리기
        fin_angles = linspace(0, 2*pi, fin_count + 1);
        fin_angles = fin_angles(1:fin_count); % 마지막 점은 시작점과 겹치므로 제외
    
        for i = 1:fin_count
            phi = fin_angles(i); % 현재 핀의 각도
    
            % 2D 좌표를 3D로 변환 및 회전
            verts_3d_rotated = zeros(4, 3);
            verts_3d_rotated(:,1) = verts_2d(:,1); % X 좌표는 동일
            verts_3d_rotated(:,2) = verts_2d(:,2) .* cos(phi); % Y = r * cos(phi)
            verts_3d_rotated(:,3) = verts_2d(:,2) .* sin(phi); % Z = r * sin(phi)
    
            % Patch로 면 그리기
            patch('Vertices', verts_3d_rotated, 'Faces', [1 2 3 4], ...
                  'FaceColor', 'red', 'EdgeColor', 'k', 'FaceAlpha', 0.9);
        end
         fprintf('        %d개의 핀을 3D로 그림 (단순화된 면).\n', fin_count);
         
         % 최대 핀 높이 반환 (화살표 크기 계산용)
         fin_max_height = finHeight;
    end
end