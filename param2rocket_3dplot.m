function param2rocket_3dplot(rocketParams)
    % 입력받은 로켓 파라미터 구조체(중첩 포함 가능)를 사용하여 3D 프로파일을 플롯합니다.
    % extractRocketParams (보강된 버전) 함수로부터 얻은 로켓 부품 파라미터 구조체 배열을 사용합니다.
    %
    % Args:
    %   rocketParams (struct array): extractRocketParams 함수로부터 얻은
    %                                로켓 부품 파라미터 구조체 배열.
    
    n_theta_points = 30; % 3D 회전 시 원주 방향 점 개수 (기본값)
    
    % --- 입력 파라미터 유효성 검사 ---
    % 셀 배열 입력도 허용하지만, 내부적으로 구조체 배열로 변환 시도
     if (~isstruct(rocketParams) && ~iscell(rocketParams)) || isempty(rocketParams) 
        error('입력 인수는 비어 있지 않은 구조체 배열 또는 셀 배열이어야 합니다 (extractRocketParams 결과).');
     end
     
     % 입력이 셀 배열이면 구조체 배열로 변환 시도
     if iscell(rocketParams) 
        try
            % 셀 배열 내의 구조체들을 하나로 합침
            rocketParams = [rocketParams{:}];
        catch ME_struct
             error('입력 셀 배열을 구조체 배열로 변환하는 데 실패했습니다: %s', ME_struct.message);
        end
     end
     
     % 최종적으로 구조체 배열이며 비어있지 않은지 확인
     if ~isstruct(rocketParams) || isempty(rocketParams) 
          error('입력 구조체 배열이 유효하지 않은 형식입니다. 비어 있지 않은 구조체 배열이 필요합니다.');
     end
     
     % 구조체 배열의 첫 번째 요소에서 필수 필드 존재 여부 확인 (최소한의 검사)
     if ~isfield(rocketParams(1), 'Type') || ~isfield(rocketParams(1), 'Position') || ...
        ~isfield(rocketParams(1), 'Geometry') || ~isfield(rocketParams(1), 'Material') || ...
        ~isfield(rocketParams(1), 'InstanceInfo') 
          error('입력 구조체 배열의 형식이 예상과 다릅니다. 필수 필드(Type, Position, Geometry, Material, InstanceInfo 등)가 누락되었습니다.');
     end

    % --- 내부 그리기 함수 호출 ---
    % 모든 실제 플로팅 로직은 이 중첩 함수에서 처리
    drawRocketProfile3D(rocketParams, n_theta_points);
    
end % end of param2rocket_3dplot


% =========================================================================
% Nested Main Drawing Function - rocketParams 구조체를 입력으로 받음
% 보강된 extractRocketParams 구조체에 맞춰 수정됨
% =========================================================================
function drawRocketProfile3D(rocketParams, n_theta)
    %fprintf('\nOpenRocket 3D Profile Plotter V4.3 (Minimal Fix for Nested Fins)\n'); % 버전 정보 업데이트
    fprintf('\nRocket 3D Profile Plotter (compatible with enhanced extraction)\n');
    fprintf('====================================================================\n');

    % --- 로켓 기본 정보 (구조체에서 가져오기 시도) ---
    % 로켓 이름 태그 (<rocket><name>...)는 최상위 레벨에 있지만, 현재 함수는 부품 구조체만 입력으로 받음
    % 따라서 로켓 이름은 추정치 또는 기본값을 사용
    % TODO: 최상위 <rocket> 노드에서 이름 정보를 추출하여 인자로 전달받도록 개선 필요
    rocketName = 'Extracted Rocket'; % 기본값 유지
    
    fprintf('로켓 이름: %s (추정)\n', rocketName);
    
    % --- 3D 플로팅 준비 ---
    % 새 Figure 창 생성
    hFig = figure('Name', sprintf('Rocket 3D Model: %s', rocketName), 'NumberTitle', 'off');
    hAx = axes('Parent', hFig); % Figure 안에 Axes 생성
    
    % Axes 속성 설정
    hold(hAx, 'on'); % 기존 플롯 유지하며 새로 그리기
    axis(hAx, 'equal'); % XYZ 축 스케일 동일
    grid(hAx, 'on');    % 그리드 표시
    
    % 축 라벨 설정
    xlabel(hAx, 'X (Axial, m)'); 
    ylabel(hAx, 'Y (m)'); 
    zlabel(hAx, 'Z (m)'); % OpenRocket 좌표계: X=축, Y=가로, Z=위/아래
    
    % 제목 설정
    title(hAx, sprintf('로켓 3D 모델: %s', rocketName));
    
    % 3D 뷰 설정 및 회전 활성화
    view(hAx, 3);      % 3D 시점 설정
    rotate3d(hAx, 'on'); % 마우스로 3D 뷰 회전 활성화
    
    % 플롯 좌표계 설명 주석
    % OpenRocket의 Body 좌표계: +X는 앞, +Y는 오른쪽, +Z는 아래
    % MATLAB의 3D 플롯 좌표계 기본값: +X는 오른쪽, +Y는 앞, +Z는 위
    % 여기서는 MATLAB 기본 좌표계를 사용하고, OpenRocket의 +Z 방향을 MATLAB의 -Z 방향에 매핑
    % 따라서 OpenRocket의 (x, y, z) -> MATLAB의 (x, y, -z) 변환 필요
    % plot 함수들에서 Z 좌표를 반전시켜서 OpenRocket의 +Z (아래) 방향이 플롯의 +Z (위) 방향으로 매핑되도록 구현

    % --- 4. 부품 처리 로직 (추출된 구조체 배열 순회) ---
    % 최상위 레벨 부품의 절대 시작 위치는 이미 계산되어 있으므로, 순차적 처리는 필요 없음
    % 하위 부품의 경우, 부모의 절대 위치를 기준으로 계산된 절대 위치를 사용
    
    % 로켓 전체 크기 추정용 변수 (좌표축 및 뷰 설정에 사용)
    rocket_max_length = 0; 
    rocket_max_radius = 0; 

    fprintf('\n--- 부품 처리 시작 (3D from Params) ---\n');
    fprintf('처리할 최상위 부품 수: %d개\n', numel(rocketParams)); % 최상위 부품 수만 표시

    % 최상위 부품 배열을 순회하며 각 부품과 그 하위 부품을 처리
    for i = 1:numel(rocketParams) 
        currentComponent = rocketParams(i); % 구조체 배열 요소 접근 () 사용

        % 부품의 필수 필드 유효성 재확인 (입력 유효성 검사 후 한번 더)
        if ~isstruct(currentComponent) || ~isfield(currentComponent, 'Type') || ~isfield(currentComponent, 'Name') || ...
           ~isfield(currentComponent,'Position') || ~isfield(currentComponent.Position,'AbsoluteStartX') || ...
           ~isfield(currentComponent, 'Geometry') || ~isfield(currentComponent, 'Material') || ...
           ~isfield(currentComponent, 'InstanceInfo') 
             fprintf('>> 경고: [%d]번째 부품 데이터가 유효하지 않아 건너<0xEB><0x8B>니다.\n', i);
             continue; % 유효하지 않은 부품은 건너뛰기
        end
        
        % 부품 기본 정보 가져오기
        compName = currentComponent.Name;
        nodeName = currentComponent.Type;
        absolute_start_x = currentComponent.Position.AbsoluteStartX;
        
        % 위치가 유효하지 않으면 건너뛰기
        if isnan(absolute_start_x)
            fprintf('>> 경고: 부품 "%s" (%s)의 시작 위치가 NaN이므로 건너<0xEB><0x8B>니다.\n', compName, nodeName);
            continue;
        end

        % 현재 부품을 그리기 위한 시작 반경 결정 (YZ 평면 기준)
        % Transition은 ForeRadius, Bodytube는 Radius 등을 사용
        % 하위 부품 그릴 때는 부모 튜브의 해당 위치 반경을 사용 (Bodytube 반경, Transition 끝 반경 등)
        % 최상위 부품의 경우, 이전 최상위 부품의 끝 반경을 사용할 수도 있으나, AbsoluteStartX가 순차적이지 않을 수 있으므로
        % 해당 부품 자체의 시작 반경 정보를 사용하거나 0에서 시작한다고 가정하는 것이 일반적.
        % 예를 들어, Nosecone은 0에서 시작, Bodytube/Transition은 자신의 Radius/ForeRadius에서 시작.
        % 핀은 부모 Bodytube의 반경에서 시작.
        
        % 여기서는 각 부품 타입별로 그리기 시작 반경을 결정하는 헬퍼 함수를 호출
        % 이 헬퍼 함수는 부품 자체의 반경 정보나 부모의 반경 정보를 고려하여 적절한 시작 반경을 반환
        % 최상위 부품의 경우 '부모 반경'은 0으로 전달
        parent_radius_for_plotting = 0; % 최상위 부품의 부모 반경은 0

        fprintf('\n>> [%d] 처리 시작: %s (%s) at X=%.4f\n', i, compName, nodeName, absolute_start_x);

        component_len = 0; % 이 부품의 축 방향 길이 (플롯에 사용)
        % component_end_radius = 0; % 이 부품의 끝 반경 (하위 부품에게 부모 반경으로 전달) - 아래 switch에서 설정

        try
            % --- 보강된 구조체의 필드 접근 예시 (플롯 자체에는 사용 안 함) ---
            % 이 정보들은 CM/MOI 계산이나 더 상세한 플롯 (예: 질량 분포 시각화, 핀 두께 플롯)에 사용 가능
            % fprintf('      Instance Info: Count=%d, RadialPos=%.4f, AngleOffset=%.4f, Rotation=%.4f\n', ...
            %     currentComponent.InstanceInfo.Count, currentComponent.InstanceInfo.RadialPosition, currentComponent.InstanceInfo.AngleOffset, currentComponent.InstanceInfo.Rotation);
            % if isfield(currentComponent.Position, 'IsCGOverridden') && currentComponent.Position.IsCGOverridden
            %      fprintf('      CG Override: Enabled, Value=%.4f\n', currentComponent.Position.OverrideCGX);
            % end
            % if isfield(currentComponent, 'MotorMount') && isfield(currentComponent.MotorMount, 'HasMount') && currentComponent.MotorMount.HasMount
            %     fprintf('      Motor: %s %s (D=%.4f, L=%.4f)\n', ...
            %          currentComponent.MotorMount.Motor.Manufacturer, currentComponent.MotorMount.Motor.Designation, ...
            %          currentComponent.MotorMount.Motor.Diameter, currentComponent.MotorMount.Motor.Length);
            % end
            % if isfield(currentComponent, 'Material') && isfield(currentComponent.Material, 'Type')
            %    fprintf('      Material: Name="%s", Type="%s", Density=%.4f\n', ...
            %         currentComponent.Material.Name, currentComponent.Material.Type, currentComponent.Material.Density);
            % end
            % --- 보강된 구조체의 필드 접근 예시 끝 ---


            % --- 부품 타입별 그리기 로직 ---
            switch nodeName
                case 'nosecone'
                    geo = currentComponent.Geometry;
                    % Geometry 필드가 있는지, 필요한 서브 필드가 있는지 안전하게 체크
                    if ~isfield(geo, 'Length') || ~isfield(geo, 'AftRadius') || ~isfield(geo, 'Shape') || ...
                       isnan(geo.Length) || isnan(geo.AftRadius)
                       error('Nosecone 필수 지오메트리 누락 또는 유효하지 않음');
                    end
                    len = geo.Length; 
                    aftRadius = geo.AftRadius; 
                    shapeType = geo.Shape; 
                    % shapeParam은 선택적이므로 안전하게 가져옴
                    shapeParam = NaN; if isfield(geo, 'ShapeParameter'), shapeParam = geo.ShapeParameter; end
                    % Haack shapeParameter 기본값은 0 (LV-Haack)
                    if isnan(shapeParam), shapeParam = 0; end 
                    
                    color = 'blue'; % 노즈콘 색상

                    % plotNoseCone3D 호출 (시작 반경은 0으로 가정하고, 끝 반경(R)과 길이(L)로 형상 결정)
                    % 노즈콘은 보통 뾰족한 끝(반경 0)에서 시작
                    plotNoseCone3D(shapeType, shapeParam, len, aftRadius, absolute_start_x, color, n_theta);
                    
                    component_len = len; % 노즈콘 본체 길이
                    component_end_radius = aftRadius; % 기본 끝 반경은 AftRadius

                    % 노즈콘 어깨 정보 고려 (어깨가 있으면 어깨 끝 반경과 길이를 사용)
                    if isfield(geo, 'AftShoulderLength') && ~isnan(geo.AftShoulderLength) && geo.AftShoulderLength > 0 && ...
                       isfield(geo, 'AftShoulderRadius') && ~isnan(geo.AftShoulderRadius)
                          component_len = component_len + geo.AftShoulderLength; % 어깨 길이만큼 전체 길이 증가
                          component_end_radius = geo.AftShoulderRadius; % 끝 반경은 어깨 반경
                          % TODO: 노즈콘 어깨 부분 3D 형상 추가 그리기 (원통)
                           fprintf('    (어깨 길이: %.3f, 반경: %.3f)', geo.AftShoulderLength, geo.AftShoulderRadius);
                    end
                    % 전체 로켓 최대 반경 업데이트 (노즈콘의 가장 넓은 부분 또는 어깨 반경 사용)
                    rocket_max_radius = max(rocket_max_radius, component_end_radius);


                case 'bodytube'
                    geo = currentComponent.Geometry;
                     % 필수 지오메트리 필드 안전하게 체크
                     if ~isfield(geo, 'Length') || ~isfield(geo, 'Radius') || isnan(geo.Length) || isnan(geo.Radius)
                       error('Bodytube 필수 지오메트리 누락 또는 유효하지 않음');
                    end
                    len = geo.Length; 
                    tube_radius = geo.Radius;
                    
                    color = [0.3 0.3 0.3]; % 진회색 바디튜브 색상

                    % plotBodyTube3D 호출 (시작 X 위치, 길이, 반경 전달)
                    plotBodyTube3D(absolute_start_x, len, tube_radius, color, n_theta);
                    
                    component_len = len; 
                    component_end_radius = tube_radius; % 바디튜브 끝 반경은 자신의 반경과 동일

                    rocket_max_radius = max(rocket_max_radius, tube_radius);

                    % --- 중첩된 Subcomponents에서 Fin Set 및 기타 부품 찾아 그리기 ---
                    % Subcomponents 필드는 추출 단계에서 항상 셀 배열로 초기화됨
                    if isfield(currentComponent, 'Subcomponents') && ~isempty(currentComponent.Subcomponents)
                        nestedCompsCell = currentComponent.Subcomponents; % Subcomponents는 셀 배열
                        fprintf('    내부 Subcomponents %d개 확인.\n', numel(nestedCompsCell));
                        
                        % 하위 부품 순회
                        for j = 1:numel(nestedCompsCell)
                            try % 개별 하위 부품 처리 오류 방지 블록
                                % 셀 배열 요소 접근은 {} 사용
                                nestedComp = nestedCompsCell{j}; 
                                
                                % 하위 컴포넌트가 유효한 구조체인지, 타입/이름 필드가 있는지 최소한의 체크
                                if ~isstruct(nestedComp) || ~isfield(nestedComp, 'Type') || ~isfield(nestedComp, 'Name')
                                     fprintf('      경고: [%d]번째 하위 부품 데이터 형식이 유효하지 않아 건너<0xEB><0x8B>니다.\n', j);
                                     continue; % 유효하지 않은 하위 부품 건너뛰기
                                end

                                % 하위 부품 타입별 그리기 로직 호출 (재귀적)
                                % 하위 부품의 절대 시작 위치는 이미 nestedComp.Position.AbsoluteStartX에 계산되어 있음
                                % 하위 부품에게 전달할 '부모 반경'은 현재 부모 컴포넌트(bodytube)의 반경
                                % n_theta는 그대로 전달
                                
                                % TODO: 하위 부품 그리기 함수를 별도로 만들고 재귀 호출하는 방식 고려
                                % 현재는 Fin Set만 특별 처리하고 나머지 하위 부품은 플롯하지 않음.
                                
                                % --- Fin Set (trapezoidfinset) 타입인 경우 processFinSet3D 호출 ---
                                if strcmp(nestedComp.Type, 'trapezoidfinset')
                                    fprintf('      발견된 Fin Set: %s\n', nestedComp.Name);
                                    % processFinSet3D 호출 (핀 구조체, 부모 반경(바디튜브 반경), n_theta 전달)
                                    % 핀은 부모 바디튜브의 반경에 부착되므로 tube_radius 전달
                                    [fin_max_h] = processFinSet3D(nestedComp, tube_radius, n_theta); 
                                    if ~isnan(fin_max_h)
                                        % 핀 높이를 고려하여 로켓의 전체 최대 반경 업데이트
                                        rocket_max_radius = max(rocket_max_radius, tube_radius + fin_max_h); 
                                    end
                                % TODO: 다른 하위 부품 타입 (masscomponent, parachute 등)도 필요시 플롯 로직 추가
                                % 예: masscomponent의 packedlength/radius로 간단한 원통 그리기 등
                                % elseif strcmp(nestedComp.Type, 'masscomponent')
                                %    % plotMassComponent3D(nestedComp, parent_radius_for_this_nested_item, n_theta); % 예시
                                % else
                                %     fprintf('      하위 부품 타입 %s (%s) 처리 (플롯 로직 없음).\n', nestedComp.Type, nestedComp.Name);
                                end % if strcmp(nestedComp.Type, ...)

                            catch ME_nested
                                 % 개별 하위 부품 처리 중 오류 발생 시 메시지 출력하고 다음 하위 부품으로 진행
                                 fprintf(2, '      오류: 하위 부품 "%s" (%s) 처리 중 오류 발생: %s (Line %d)\n', ...
                                     nestedComp.Name, nestedComp.Type, ME_nested.message, ME_nested.stack(1).line);
                            end % try/catch nested component
                        end % for nestedCompsCell loop

                    else
                         fprintf('    내부 Subcomponents 없음 또는 비어있음.\n');
                    end % if Subcomponents exists

                case 'transition'
                    geo = currentComponent.Geometry;
                     % 필수 지오메트리 필드 안전하게 체크
                     if ~isfield(geo, 'Length') || ~isfield(geo, 'ForeRadius') || ~isfield(geo, 'AftRadius') || ...
                        isnan(geo.Length) || isnan(geo.ForeRadius) || isnan(geo.AftRadius)
                       error('Transition 필수 지오메트리 누락 또는 유효하지 않음');
                    end
                    len = geo.Length; 
                    foreRadius = geo.ForeRadius; 
                    aftRadius = geo.AftRadius;
                    shapeType = 'conical'; % Transition은 대부분 원뿔형 (XML에 shape 필드도 있음)
                    color = 'magenta'; % 트랜지션 색상
                    
                    fprintf('    형태: %s, 길이: %.3f, Fore반경: %.3f, Aft반경: %.3f\n', shapeType, len, foreRadius, aftRadius);
                    
                    % plotTransition3D 호출 (시작 X, 길이, 시작 반경, 끝 반경 전달)
                    plotTransition3D(absolute_start_x, len, foreRadius, aftRadius, color, n_theta);
                    
                    component_len = len; 
                    component_end_radius = aftRadius; % 트랜지션 끝 반경은 AftRadius

                    % 전체 로켓 최대 반경 업데이트 (트랜지션의 최대 반경 사용)
                    rocket_max_radius = max(rocket_max_radius, max(foreRadius, aftRadius));

                case 'trapezoidfinset'
                     % 최상위 레벨에 핀셋이 있는 경우는 흔치 않지만, 만약 있다면 메시지 출력하고 그리지 않음
                     fprintf('    Fin Set 타입 (%s) - 최상위 레벨에서는 그리지 않음 (일반적으로 Bodytube 하위에 위치).\n', compName);
                     component_len = 0; % 최상위 핀셋 자체는 축방향 길이 기여 없음
                     % component_end_radius는 부모 반경을 따라가야 하지만, 최상위 부모 반경(0)은 의미 없음
                     % 이 부품 이후의 순차적 위치/반경에 영향 주지 않음 (CM/MOI 계산 시에만 영향)
                     component_end_radius = 0; % 기본값 (의미 없음)

                case {'masscomponent', 'parachute', 'tubecoupler', 'centeringring', 'bulkhead'}
                     % 이 컴포넌트들은 현재 3D 형상 플롯 로직이 없음
                     fprintf('    타입 %s (%s) 처리 (3D 형상 생략).\n', nodeName, compName);
                     
                     % 구조체에 저장된 길이/두께 정보를 사용하여 컴포넌트 길이 결정 (주로 AbsoluteEndX 계산에 사용되지만 플롯에서도 활용 가능)
                     component_len = 0;
                     if isfield(currentComponent.Position, 'Length') && ~isnan(currentComponent.Position.Length)
                         component_len = currentComponent.Position.Length;
                     % Position.Length는 추출 단계에서 Geometry의 PackedLength 또는 Length/Thickness를 가져와서 저장됨
                     end
                     
                     % 이 부품들의 끝 반경 (주로 부모 반경을 유지하거나 특정 반경을 가짐)
                     % CM/MOI 계산 시 필요하지만, 플롯에서는 형상이 없으므로 끝 반경 업데이트는 불필요.
                     component_end_radius = 0; % 기본값 (의미 없음)


                case 'stage'
                    % Stage는 그룹핑 컴포넌트이므로 자체적인 3D 형상은 없음
                     fprintf('    타입 Stage (%s) 처리 (그룹핑 컴포넌트, 3D 형상 없음).\n', compName);
                     component_len = 0; % Stage 자체의 명시적 길이 기여 없음
                     % Stage의 끝 반경은 하위 부품에 의해 결정되지만, 플롯 목적상 명시적 형상이 없으므로 0 유지
                     component_end_radius = 0; % 기본값 (의미 없음)

                otherwise
                    % 지원되지 않는 부품 타입
                    fprintf('    지원되지 않는 부품 타입: %s (%s). 3D 형상 없음.\n', nodeName, compName);
                    
                    % 가능하면 길이 정보만 가져와서 전체 로켓 길이 추정에는 반영
                    component_len = 0;
                    if isfield(currentComponent.Position, 'Length') && ~isnan(currentComponent.Position.Length)
                        component_len = currentComponent.Position.Length;
                    end
                    component_end_radius = 0; % 기본값 (의미 없음)

            end % switch nodeName
            
            % 전체 로켓 길이 업데이트 (가장 마지막 부품의 끝 위치가 로켓 길이가 될 가능성 높음)
            % 각 부품의 절대 끝 위치 (AbsoluteStartX + component_len) 중 최대값을 사용
            if isfield(currentComponent.Position, 'AbsoluteEndX') && ~isnan(currentComponent.Position.AbsoluteEndX)
                 rocket_max_length = max(rocket_max_length, currentComponent.Position.AbsoluteEndX);
            else % AbsoluteEndX가 없는 경우 (Position 필드가 없거나 계산 실패 등)
                 rocket_max_length = max(rocket_max_length, absolute_start_x + component_len);
            end


        catch ME_comp
             % 개별 부품 처리 중 오류 발생 시 메시지 출력하고 다음 부품으로 진행
             fprintf(2, '    오류: 부품 "%s" (%s) 처리 중 오류 발생: %s (Line %d)\n', ...
                 compName, nodeName, ME_comp.message, ME_comp.stack(1).line);
             % 오류 발생 시 해당 부품의 길이/반경 정보는 무시하고 다음 부품으로 진행
             % component_len과 component_end_radius는 초기값(0) 유지됨
        end % try/catch component

    end % for rocketParams loop (최상위 부품 순회)


    % --- 좌표축 그리기 ---
    % 플롯된 형상이 없을 경우 또는 길이가 0일 경우 기본 크기 설정
    if isempty(rocket_max_length) || rocket_max_length <= 0, rocket_max_length = 1; end
    % 반경이 0이거나 없을 경우 기본 크기 설정
    if isempty(rocket_max_radius) || rocket_max_radius <= 0, rocket_max_radius = 0.1; end
    
    % 좌표축 길이 계산 (플롯 범위의 약 15% ~ 2배 사용)
    arrow_length_x = rocket_max_length * 1.15; 
    % YZ축은 반경 기반으로 하되, 너무 작으면 최소값 보장
    arrow_length_yz = max(rocket_max_radius * 2.5, 0.25); % 반경의 2.5배, 최소 0.25m
    max_head_size = 0.1; % 화살표 머리 크기 (비율)

    % 좌표축 시작점 (원점)
    origin = [0, 0, 0];

    % X축 (빨강) - 로켓의 축 방향 (앞)
    quiver3(hAx, origin(1), origin(2), origin(3), arrow_length_x, 0, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', max_head_size); 
    text(hAx, arrow_length_x * 1.05, 0, 0, 'X (Axial)', 'Color', 'r', 'FontSize', 10, 'FontWeight', 'bold');

    % Y축 (녹색) - 로켓의 가로 방향 (오른쪽)
    quiver3(hAx, origin(1), origin(2), origin(3), 0, arrow_length_yz, 0, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', max_head_size); 
    text(hAx, 0, arrow_length_yz * 1.05, 0, 'Y', 'Color', 'g', 'FontSize', 10, 'FontWeight', 'bold');

    % Z축 (파랑) - 로켓의 상/하 방향 (MATLAB 플롯에서는 보통 위쪽)
    quiver3(hAx, origin(1), origin(2), origin(3), 0, 0, arrow_length_yz, 0, 'b', 'LineWidth', 2, 'MaxHeadSize', max_head_size); 
    text(hAx, 0, 0, arrow_length_yz * 1.05, 'Z', 'Color', 'b', 'FontSize', 10, 'FontWeight', 'bold');

    fprintf('\n좌표축을 (%.4f, %.4f, %.4f) 위치에 추가했습니다 (X:빨강, Y:녹색, Z:파랑).\n', origin);
    fprintf('주의: 플롯의 Z축은 위쪽(Up)이며, OpenRocket 내부 Body 좌표계의 +Z축(Down)과 방향이 반대입니다.\n');
    fprintf('      따라서 OpenRocket의 +Z 방향 부품(예: 아래쪽 핀의 Z 좌표)은 플롯에서 아래쪽에 그려집니다.\n');


    % --- 5. 플롯 마무리 ---
    % hold(hAx, 'off'); % Hold 해제
    hold on
    
    % 플롯 범위 자동 설정 (데이터 기반)
    axis(hAx, 'tight'); 
    
    % 축 비율 동일 설정
    daspect(hAx, [1 1 1]); 
    
    % 조명 효과 추가
    camlight(hAx, 'left'); 
    lighting(hAx, 'gouraud'); 
    material(hAx, 'dull');
    
    % 뷰를 다시 3D 기본값으로 설정 (rotate3d on 해도 가끔 평면뷰로 바뀔 때 대비)
    view(hAx, 3);
    
    fprintf('\n3D 모델 그리기가 완료되었습니다 (파라미터 기반).\n');

end % end of drawRocketProfile3D


% =========================================================================
% Nested Helper Functions for Drawing - 보강된 구조체에 맞춰 안전성 보강
% =========================================================================

% Helper: 현재 컴포넌트의 시작 위치에서의 로켓 반경을 추정 (플롯용)
% 재귀 호출 시 parent_radius 인자와는 별개로, 현재 부품을 그릴 때 어느 반경에서 시작해야 할지를 결정
% (Transition의 ForeRadius, Bodytube의 Radius 등)
% 이 함수는 주로 상위 함수에서 부품을 그릴 때 YZ 평면의 시작 반경을 결정하는 데 사용됩니다.
function start_radius = getStartRadiusForPlotting(componentStruct, default_radius_from_parent)
    % 기본값은 부모로부터 전달받은 반경 (최상위 부품의 경우 0)
    start_radius = default_radius_from_parent; 

    % Geometry 필드가 없거나 구조체가 아니면 기본값 사용
    if ~isfield(componentStruct, 'Geometry') || ~isstruct(componentStruct.Geometry)
        return; 
    end
    geo = componentStruct.Geometry;

    % 부품 타입별로 시작 반경을 결정하는 주요 필드를 찾습니다.
    % 'auto' 처리는 이미 추출 단계에서 default_radius_from_parent로 반영되었어야 함
    switch componentStruct.Type
        case 'nosecone'
            % plotNoseCone3D는 0부터 시작하여 AftRadius까지 그립니다. 시작 반경 자체를 그리지 않음.
            % 이 헬퍼 함수는 주로 다른 컴포넌트의 '시작' 반경을 결정하므로, 노즈콘에는 적용하기 모호.
            % 일단 부모 반경(default_radius_from_parent)을 기본으로 반환합니다.
             start_radius = default_radius_from_parent; 

        case 'bodytube'
            % 바디튜브의 시작 반경은 자신의 Radius
            if isfield(geo, 'Radius') && ~isnan(geo.Radius)
                 start_radius = geo.Radius;
            end

        case 'transition'
            % 트랜지션의 시작 반경은 ForeRadius
            if isfield(geo, 'ForeRadius') && ~isnan(geo.ForeRadius)
                start_radius = geo.ForeRadius;
            end

        case 'trapezoidfinset'
            % 핀은 부모 튜브 반경에 부착되므로 부모 반경(default_radius_from_parent)이 중요
            start_radius = default_radius_from_parent; 

        case 'tubecoupler'
            % 튜브 커플러의 시작 반경은 OuterRadius
            if isfield(geo, 'OuterRadius') && ~isnan(geo.OuterRadius)
                start_radius = geo.OuterRadius;
            end
        
        case {'centeringring', 'bulkhead'}
             % 센터링 링이나 벌크헤드의 외부 반경
             if isfield(geo, 'OuterRadius') && ~isnan(geo.OuterRadius)
                start_radius = geo.OuterRadius;
             end

        case {'masscomponent', 'parachute', 'stage'}
            % 이 컴포넌트들은 자체적인 메인 반경을 가지기보다 부모 안에 위치하거나 그룹핑 역할
            % 부모 반경(default_radius_from_parent)을 유지
            start_radius = default_radius_from_parent;

        otherwise
            % 기타 타입은 기본 반경 유지
            start_radius = default_radius_from_parent;
    end
    
    % Note: InstanceInfo.RadialPosition은 컴포넌트의 형상 자체 반경이 아니라,
    % 부모 반경으로부터의 추가적인 오프셋이므로 여기서 start_radius를 수정하지 않습니다.
    % 핀 그리기와 같이 해당 오프셋을 반영해야 하는 컴포넌트 타입에서 개별적으로 적용해야 합니다.

end % end of getStartRadiusForPlotting


% Helper: Nosecone 그리기 (기존 로직 유지, Geometry 필드 접근 안전성 보강)
% L: Length, R: AftRadius
function plotNoseCone3D(shapeType, shapeParam, L, R, start_x, color, n_theta)
    n_points = 50; % 형상 곡선을 그릴 점 개수
    x_local = linspace(0, L, n_points); % 노즈콘 로컬 X 좌표 (0부터 L까지)
    y_local = zeros(1, n_points);     % 해당 X에서의 반경 (로컬 Y)

    % 기본 유효성 검사: 길이와 끝 반경은 0보다 커야 함
    if L <= 0 || R <= 0
         fprintf('      경고: Nosecone 길이(%.4f) 또는 끝 반경(%.4f)이 0 이하입니다. 그리기 생략.\n', L, R);
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
                 % 유효하지 않은 Power 지수 (0 이하 또는 1 초과)의 경우 기본값 0.5 사용
                 if isnan(n_pow) || n_pow <= 0 || n_pow > 1 
                     n_pow = 0.5; 
                 end
                 y_local = R * (x_local / L).^n_pow;
             case 'haack' % Haack Series 형태 (k=shapeParam)
                  k = shapeParam; % shapeParam은 Haack 상수 k
                  % 유효하지 않은 Haack 상수 (보통 -1에서 1 사이 값)
                  if isnan(k) || abs(k) > 1 
                       k = 0; % 유효하지 않으면 k=0 (LV-Haack) 사용
                  end
                  
                  % LV-Haack 공식 (k=0) 및 일반 Haack 공식
                  if k == 0 % LV-Haack (정적 저항 최소화)
                       % acos 입력값이 -1과 1 범위를 벗어나지 않도록 클램핑
                       theta = acos(max(-1, min(1, 1 - 2*x_local/L))); 
                       y_local = (R / sqrt(pi)) * sqrt(max(0, theta - 0.5*sin(2*theta)));
                  else % 일반 Haack 공식 (압력 저항 최소화) - OpenRocket 구현과 약간 다를 수 있음
                       theta = acos(max(-1, min(1, 1 - 2*x_local/L)));
                       y_local = (R / sqrt(pi)) * sqrt(max(0, theta - (k/2)*sin(theta))); 
                  end
             otherwise % 지원되지 않는 형태는 원뿔형으로 근사
                 fprintf('      주의: 지원되지 않는 Nosecone 형태 "%s"입니다. 원뿔형으로 근사하여 그립니다.\n', shapeType);
                 y_local = R * (x_local / L);
         end
        
        % 계산된 로컬 Y 값 정리 (NaN, 복소수 결과, 음수 방지 및 시작/끝점 보장)
        y_local(isnan(y_local) | ~isreal(y_local)) = 0; % NaN 또는 복소수 결과를 0으로 처리
        y_local = max(0, y_local); % 음수 반경을 0으로 처리
        y_local(1) = 0; % 로컬 X=0 (노즈콘 시작점)에서의 반경은 항상 0 보장
        y_local(end) = R; % 로컬 X=L (노즈콘 끝점)에서의 반경은 항상 AftRadius (R) 보장


    catch ME
         fprintf(2, '      오류: 노즈콘 형상 계산 중 오류 발생: %s\n', ME.message); 
         return; % 오류 발생 시 그리기 중단
    end
    
    % 3D 표면 생성 (원주 방향으로 회전)
    theta = linspace(0, 2*pi, n_theta); % 0부터 2*pi까지 n_theta 등분 각도 생성
    
    % X 좌표: 로컬 X + 시작 X 위치
    X_surf = start_x + repmat(x_local(:), 1, n_theta); 
    
    % Y, Z 좌표: 로컬 Y (반경)를 이용하여 원주 따라 회전
    % Y_3D = r * cos(theta), Z_3D = r * sin(theta)
    Y_surf = y_local(:) * cos(theta); 
    Z_surf = y_local(:) * sin(theta); 
    
    % Z축 방향 반전: OpenRocket Body 좌표계의 +Z(아래)를 MATLAB 플롯의 +Z(위)로 매핑하기 위해 Z 좌표 반전
    Z_surf = -Z_surf;

    % 표면 플롯
    surf(X_surf, Y_surf, Z_surf, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 1.0);

end % end of plotNoseCone3D


% Helper: Bodytube 그리기 (기존 로직 유지, Geometry 필드 접근 안전성 보강)
function plotBodyTube3D(start_x, len, radius, color, n_theta)
    % 기본 유효성 검사: 길이와 반경은 0보다 커야 함
    if len <= 0 || radius <= 0
         fprintf('      경고: Bodytube 길이(%.4f) 또는 반경(%.4f)이 0 이하입니다. 그리기 생략.\n', len, radius);
         return; 
    end

    % 3D 표면 생성 (원통)
    % cylinder 함수는 단위 원통을 생성 (반경 1, 높이 1)
    % [X_unit, Y_unit, Z_unit] = cylinder(R, N) -> X_unit, Y_unit은 단위 원, Z_unit은 높이 (0~1)
    % R=1, N=n_theta-1개의 세그먼트 사용
    [X_unit, Y_unit, Z_unit] = cylinder(1, n_theta - 1); 

    % 단위 원통을 실제 바디튜브 크기로 스케일 및 이동
    % X_surf: 로켓 축 방향 위치. 단위 원통의 높이(Z_unit, 0~1)를 바디튜브 길이(len)로 스케일하고, 바디튜브 시작 X 위치(start_x)만큼 이동.
    X_surf = start_x + Z_unit * len; 
    
    % Y_surf, Z_surf: 로켓 단면의 Y, Z 좌표. 단위 원통의 X_unit, Y_unit 좌표를 바디튜브 반경(radius)으로 스케일.
    Y_surf = Y_unit * radius;       
    Z_surf = X_unit * radius;       

     % Z축 방향 반전: OpenRocket +Z down -> MATLAB +Z up
     % cylinder 함수가 생성하는 단위 원통의 좌표계와 MATLAB 플롯의 Z축 방향에 따라 Z 좌표 반전 필요
     % 일반적으로 cylinder의 X_unit, Y_unit이 XY 평면의 원을 만들고 Z_unit이 높이를 나타냅니다.
     % surf(X, Y, Z) 호출 시 Z_unit을 X_surf에 사용했으므로, Y_unit과 X_unit이 단면을 만듭니다.
     % OpenRocket의 YZ 단면을 MATLAB의 YZ 평면에 그리려면 Y_surf = r*cos(theta), Z_surf = r*sin(theta) 형태가 되어야 합니다.
     % cylinder 함수는 X_unit = cos(theta), Y_unit = sin(theta) 형태로 좌표를 생성합니다.
     % 따라서 Z_surf = X_unit * radius; Y_surf = Y_unit * radius; 로 사용하면 OpenRocket의 Z에 X_unit이, Y에 Y_unit이 매핑됩니다.
     % OpenRocket의 +Z는 아래 방향이므로, 이를 MATLAB 플롯의 +Z (위) 방향과 반대로 맞추려면 Z_surf를 반전해야 합니다.
    Z_surf = -Z_surf;

    % 표면 플롯
    surf(X_surf, Y_surf, Z_surf, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 1.0);

end % end of plotBodyTube3D


% Helper: Transition 그리기 (기존 로직 유지, Geometry 필드 접근 안전성 보강)
function plotTransition3D(start_x, len, start_radius, end_radius, color, n_theta)
    % 기본 유효성 검사: 길이와 반경은 유효해야 함 (길이 > 0, 반경 >= 0)
    if len <= 0 || start_radius < 0 || end_radius < 0
         fprintf('      경고: Transition 길이(%.4f) 또는 반경(%.4f, %.4f)이 유효하지 않습니다. 그리기 생략.\n', len, start_radius, end_radius);
         return; 
    end

    % 원뿔대 옆면 생성 (두 개의 원을 잇는 방식)
    x_local = [0, len]; % 로컬 X 좌표 (트랜지션 시작=0, 끝=len)
    r_profile = [start_radius, end_radius]; % 해당 X 좌표에서의 반경
    
    theta = linspace(0, 2*pi, n_theta); % 원주 각도 (0부터 2*pi까지 n_theta 등분)

    % 3D 표면 좌표 계산
    % X_surf: 로켓 축 방향 위치. 로컬 X를 로켓 시작 X 위치(start_x)만큼 이동. repmat을 사용하여 모든 원주 각도에 대해 동일한 X 좌표 적용.
    X_surf = start_x + repmat(x_local(:), 1, n_theta); 
    
    % Y_surf, Z_surf: 로켓 단면의 Y, Z 좌표. 해당 X에서의 반경(r_profile)을 이용하여 원주 따라 회전.
    % Y_3D = r * cos(theta), Z_3D = r * sin(theta)
    Y_surf = r_profile(:) * cos(theta); 
    Z_surf = r_profile(:) * sin(theta); 

    % Z축 방향 반전: OpenRocket +Z down -> MATLAB +Z up
    Z_surf = -Z_surf;

    % 표면 플롯
    surf(X_surf, Y_surf, Z_surf, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 1.0);

end % end of plotTransition3D


% Helper: Fin Set 그리기 (보강된 구조체에 맞춰 안전성 보강 및 오류 해결)
% 핀셋 구조체와 부모 튜브 반경, 점 개수를 받음
function [fin_max_height] = processFinSet3D(finComponentStruct, parent_radius, n_theta) %#ok<INUSD> % n_theta는 현재 핀 그리기에는 사용 안 함
    fin_max_height = NaN; % 핀 높이 반환 (최대 반경 계산용)
    finName = 'Unknown FinSet'; % 기본 이름
    if isfield(finComponentStruct, 'Name'), finName = finComponentStruct.Name; end % 이름 안전하게 가져오기

    % fprintf('        처리 중 Fin Set: %s\n', finName); % 하위 부품 순회에서 이미 출력

    % --- Geometry 정보 안전하게 가져오기 ---
    if ~isfield(finComponentStruct, 'Geometry') || ~isstruct(finComponentStruct.Geometry)
        fprintf('        경고: Fin Set "%s"에 Geometry 정보가 누락되었습니다. 그리기 생략.\n', finName);
        return;
    end
    geo = finComponentStruct.Geometry;

    % --- 핀 형상에 필요한 지오메트리 필드 안전하게 가져오기 ---
    % isfield 체크와 함께 기본값 또는 유효성 검사
    % FinCount는 Geometry에 저장된 핀 형상의 개수
    fincount = 1; % 기본 인스턴스 수
    if isfield(geo, 'FinCount') && ~isnan(geo.FinCount) && geo.FinCount >= 1 
         fincount = geo.FinCount;
    else
         % Geometry.FinCount가 없으면 InstanceInfo.Count를 시도하거나 기본값 1 사용
         if isfield(finComponentStruct, 'InstanceInfo') && isfield(finComponentStruct.InstanceInfo, 'Count') && ~isnan(finComponentStruct.InstanceInfo.Count) && finComponentStruct.InstanceInfo.Count >= 1
              fincount = finComponentStruct.InstanceInfo.Count; % InstanceInfo.Count 사용
              % fprintf('        주의: Fin Set "%s"의 Geometry.FinCount 누락. InstanceInfo.Count (%.0f) 사용.\n', finName, fincount);
         else
              fprintf('        경고: Fin Set "%s"의 FinCount/InstanceInfo.Count 정보 누락 또는 유효하지 않습니다. 기본값 1 사용.\n', finName);
         end
    end

    % 핀 형상 필수/선택적 지오메트리 필드 안전하게 가져오기
    rootChord = NaN; if isfield(geo, 'RootChord') && ~isnan(geo.RootChord), rootChord = geo.RootChord; end
    tipChord = NaN; if isfield(geo, 'TipChord') && ~isnan(geo.TipChord), tipChord = geo.TipChord; end
    finHeight = NaN; if isfield(geo, 'Height') && ~isnan(geo.Height), finHeight = geo.Height; end
    sweepLength = NaN; if isfield(geo, 'Sweep') && ~isnan(geo.Sweep), sweepLength = geo.Sweep; end
    thickness = NaN; if isfield(geo, 'Thickness') && ~isnan(geo.Thickness), thickness = geo.Thickness; end % 두께는 그리기에는 현재 사용 안 함

    % --- 형상 계산에 필요한 필수 지오메트리 유효성 검사 ---
    if isnan(rootChord) || isnan(finHeight) || rootChord <= 0 || finHeight <= 0
         fprintf('        경고: Fin Set "%s"의 필수 지오메트리(RootChord:%.4f, Height:%.4f) 누락 또는 유효하지 않음. 그리기 생략.\n', finName, rootChord, finHeight);
         fin_max_height = finHeight; % 가능한 경우 높이 반환 (NaN일 수도 있음)
         return; % 필수 정보 누락 시 그리기 중단
    end
    % 선택적 지오메트리 기본값 처리 (NaN이면 0으로 간주)
    if isnan(tipChord), tipChord = 0; end 
    if isnan(sweepLength), sweepLength = 0; end 

    % --- Position 정보 안전하게 가져오기 ---
     if ~isfield(finComponentStruct, 'Position') || ~isstruct(finComponentStruct.Position) || ~isfield(finComponentStruct.Position, 'AbsoluteStartX') || isnan(finComponentStruct.Position.AbsoluteStartX)
         fprintf('        경고: Fin Set "%s"의 위치 정보(AbsoluteStartX) 누락 또는 유효하지 않음. 그리기 생략.\n', finName);
         fin_max_height = finHeight; % 가능한 경우 높이 반환
         return; % 위치 정보 누락 시 그리기 중단
     end
    ref_coord_x = finComponentStruct.Position.AbsoluteStartX; % 핀의 부착 위치 (로켓 축 기준 절대 좌표)

    % --- 3D 위치/방향 정보 안전하게 가져오기 (플롯 로직에 적용) ---
    % InstanceInfo 필드는 추출 단계에서 항상 초기화되므로 존재 체크는 필요 없지만, 내부 필드는 체크
    instanceInfo = finComponentStruct.InstanceInfo;
    
    radialPosition = 0; if isfield(instanceInfo, 'RadialPosition') && ~isnan(instanceInfo.RadialPosition), radialPosition = instanceInfo.RadialPosition; end % 로켓 중심축 기준 방사형 거리
    radialDirection = 0; if isfield(instanceInfo, 'RadialDirection') && ~isnan(instanceInfo.RadialDirection), radialDirection = instanceInfo.RadialDirection; end % 방사형 기준 방향 (각도, 도)
    angleOffset = 0; if isfield(instanceInfo, 'AngleOffset') && ~isnan(instanceInfo.AngleOffset), angleOffset = instanceInfo.AngleOffset; end % 각 인스턴스 각도 오프셋 (도)
    rotation = 0; if isfield(instanceInfo, 'Rotation') && ~isnan(instanceInfo.Rotation), rotation = instanceInfo.Rotation; end % 부품 자체 회전 (도)

    % --- 핀 형상 꼭지점 계산 (2D, 로켓 축-반경 평면 기준) ---
    % X 좌표: SweepLength와 Chord 길이로 결정
    % Radial 좌표: 부모 반경 + RadialPosition 오프셋
    x2 = ref_coord_x;           % 후방 코드 시작점 X (부착 위치)
    x1 = x2 - rootChord;        % 후방 코드 끝점 X
    x4 = x1 + sweepLength;      % 전방 코드 시작점 X (x1에서 SweepLength만큼 이동)
    x3 = x4 + tipChord;         % 전방 코드 끝점 X (x4에서 TipChord만큼 이동)

    y1_r = parent_radius + radialPosition; % 부착 반경 + 방사형 오프셋
    y2_r = parent_radius + radialPosition; % 핀의 루트는 동일 Radial 좌표
    y3_r = parent_radius + radialPosition + finHeight; % 핀의 팁 Radial 좌표
    y4_r = parent_radius + radialPosition + finHeight; % 핀의 팁은 동일 Radial 좌표

    % 2D 꼭지점 배열 [X, Radial_Distance]
    verts_2d = [x1, y1_r; 
                x2, y2_r; 
                x3, y3_r; 
                x4, y4_r]; 

    % --- 각 핀 인스턴스 그리기 (3D) ---
    % fincount에 따라 각도 계산 (도 단위로 계산 후 라디안으로 변환)
    % 기본 등분 각도 + RadialDirection + AngleOffset
    % OpenRocket의 정확한 각도 계산 로직과 다를 수 있습니다 (예: RadialDirection이 기준이 되고 Offset이 그 기준으로부터의 편차일 수 있음).
    % 여기서는 간단하게 모두 더하는 방식으로 구현합니다.
    total_instance_angles_deg = linspace(0, 360, fincount + 1); % 0부터 360도까지 fincount+1 등분
    total_instance_angles_deg = total_instance_angles_deg(1:fincount); % 마지막 360도는 첫 점(0도)과 같으므로 제외

    % 각 인스턴스의 최종 회전 각도 (도)
    fin_angles_deg_3d = total_instance_angles_deg + radialDirection + angleOffset;

    % TODO: Rotation 필드 적용 로직 추가 (핀 자체의 X축 회전)
    % 핀 면의 법선 벡터나 꼭지점 Z/Y 좌표에 Rotation 각도를 적용해야 함. 복잡함.

    for i = 1:fincount
        phi_rad = deg2rad(fin_angles_deg_3d(i)); % 해당 인스턴스의 회전 각도 (라디안)

        verts_3d_rotated = zeros(4, 3);
        verts_3d_rotated(:,1) = verts_2d(:,1); % X 좌표는 로켓 축 방향 (변화 없음)

        % 2D Radial_Distance (verts_2d(:,2))를 3D YZ 평면 좌표로 변환
        % Y_3D = r * cos(phi), Z_3D = r * sin(phi)
        verts_3d_rotated(:,2) = verts_2d(:,2) .* cos(phi_rad); % Y 좌표 (로켓 단면 Y축)
        verts_3d_rotated(:,3) = verts_2d(:,2) .* sin(phi_rad); % Z 좌표 (로켓 단면 Z축, OpenRocket 기준 아래 방향)

        % Z축 방향 반전: OpenRocket +Z down -> MATLAB +Z up
        verts_3d_rotated(:,3) = -verts_3d_rotated(:,3); 

        % 핀 면 그리기
        patch('Vertices', verts_3d_rotated, 'Faces', [1 2 3 4], 'FaceColor', 'red', 'EdgeColor', 'k', 'FaceAlpha', 0.9);

        % TODO: 핀 두께 그리기 (patch를 이용하여 양쪽 면 및 연결 면 그리기)
        % thickness 필드를 활용하여 구현해야 함. 현재는 얇은 면으로만 그립니다.

    end % for fincount loop

    fin_max_height = finHeight; % 핀 높이 반환 (최대 반경 업데이트에 사용)

end % end of processFinSet3D