function params = Params_init_HANul3()
% PARAMS_INIT_HANUL_WITHPLOTTING HANul 로켓 시뮬레이션을 위한 파라미터 초기화 및 XML 기반 플로팅
%
%   params = Params_init_HANul_WithPlotting(xmlFilePath, generate2DPlot, generate3DPlot)
%
%   Inputs:
%       xmlFilePath     (string) : 로켓 구조가 정의된 OpenRocket XML 파일 경로.
%       generate2DPlot  (logical): true이면 2D 프로파일 플롯 생성 (기본값: true).
%       generate3DPlot  (logical): true이면 3D 모델 플롯 생성 (기본값: true).
%
%   Outputs:
%       params          (struct) : 시뮬레이션 파라미터 구조체.

    fprintf('Initializing HANul parameters and plotting from XML...\n');

    % --- 입력 인자 처리 ---
    if nargin < 1 || isempty(xmlFilePath)
        xmlFilePath = 'rocket.xml'; % 기본 XML 파일 경로
        fprintf('XML 파일 경로가 지정되지 않아 기본값 "%s" 사용.\n', xmlFilePath);
    end
    if nargin < 2 || isempty(generate2DPlot)
        generate2DPlot = true; % 기본적으로 2D 플롯 생성
    end
    if nargin < 3 || isempty(generate3DPlot)
        generate3DPlot = true; % 기본적으로 3D 플롯 생성
    end

    % =====================================================================
    %   1. 시뮬레이션 파라미터 초기화 (기존 코드 유지 - 값 변경 없음)
    % =====================================================================
    params = struct(); % 파라미터 구조체 생성

    %% Environment Parameters
    params.environment.g = [0, 0, 9.81]'; % [m/s^2]
    params.environment.ro = 1.666; % [kg/m^3]
    params.environment.ro_2 = 1.24; % [kg/m^3]
    params.environment.latitude = 32.93952; % [degrees]
    params.environment.longitude = -106.92006; % [degrees]
    params.environment.altitude = 1401; % [m] Elevation
    params.environment.wind_speed = 6.0; % [m/s]
    params.environment.pressure = 1025; % [hPa]
    params.environment.temperature = 29; % [°C]
    params.environment.temp_range = [21, 35]; % [°C]
    params.environment.T0 = 288.15;   % Sea level temperature [K]
    params.environment.P0 = 101325;   % Sea level pressure [Pa]
    params.environment.a0 = 340.294;  % Sea level speed of sound [m/s]
    params.environment.ro0 = 1.225;   % Sea level density [kg/m^3]

    %% PID Parameters
    params.control.Ka_pp = [0, 0, 0]'; %Attitude P
    params.control.Ka_p = [0, 0, 0]'; % Rate P
    params.control.Ka_i = [0, 0, 0]'; % Rate I
    params.control.Ka_d = [0, 0, 0]'; % Rate D

    %% Rocket Parameters (기존 값 유지)
    params.vehicle.J_X = 0.061; % [kgm^2]
    params.vehicle.J_Y = 13.4; % [kgm^2]
    params.vehicle.J_Z = 13.4; % [kgm^2]
    params.vehicle.S_W_ref = 1.1068; % [m^2] % surface area
    params.vehicle.S_A_ref = 0.0132; % [m^2] % diameter area
    params.vehicle.m_D = 20.014; % [kg] (Dry mass)
    params.vehicle.m_W_const = 23.647; % [kg] (Wet mass)
    params.vehicle.D_ref = 0.13; % [m] % Rocket diameter
    params.vehicle.r_ref = 0.065; % [m] % Rocket radius
    params.vehicle.Lp = -0.32; % [m] % Center of Presure distance
    params.vehicle.Lt = -1.06; % [m] % Center to Nozzle distance
    params.vehicle.Lrcs = 1; % [m] % Center to RCS distance
    params.vehicle.L = 2.71; % [m] % Rocket length
    params.vehicle.RCS_T = 1; % [N] % Reaction control system thruster
    params.vehicle.J = [params.vehicle.J_X, 0, 0;
                       0, params.vehicle.J_Y, 0;
                       0, 0, params.vehicle.J_Z];

    %% Motor Parameters (CSV 기반 유지)
    try
        opts = detectImportOptions('AeroTech_M2400T.csv', 'VariableNamingRule', 'preserve');
        data_stage1 = readtable('AeroTech_M2400T.csv', opts);
        params.motor.E1M_d = data_stage1.("Mass(g)")(end)/1000; % g to kg
        time_stage1 = data_stage1.("Time(s)");
        total_time = max(time_stage1) - min(time_stage1);
        sampling_rate = 400;
        interp_point_1 = round(total_time * sampling_rate) + 1;
        new_time_stage1 = linspace(min(time_stage1), max(time_stage1), interp_point_1);
        interp_mass_stage1 = interp1(time_stage1, data_stage1.("Mass(g)")/1000, new_time_stage1, 'linear');
        interp_thrust_stage1 = interp1(time_stage1, data_stage1.("Thrust(N)"), new_time_stage1, 'linear');
        params.motor.thrust_func_stage1 = interp_thrust_stage1;
        params.motor.mass_func_stage1 = interp_mass_stage1;
        params.vehicle.m_W = params.motor.mass_func_stage1(1); % 이 부분은 나중에 XML 기반으로 수정 필요
        params.motor.Ti_1 = interp_point_1;
    catch ME_motor
         fprintf('경고: 모터 CSV 파일 처리 중 오류 발생. 모터 파라미터가 불완전할 수 있습니다.\n (%s)\n', ME_motor.message);
         % 필요한 경우 기본값 설정
         params.motor.thrust_func_stage1 = @(t) 0;
         params.motor.mass_func_stage1 = @(t) 0;
         params.vehicle.m_W = 0;
         params.motor.Ti_1 = 0;
    end

    %% Sensor/Noise Parameters
    params.Qnoise.s_px = 0; params.Qnoise.s_py = 0; params.Qnoise.s_pz = 0;
    params.Qnoise.s_u = 0.01; params.Qnoise.s_v = 0.01; params.Qnoise.s_w = 0.01;
    params.Qnoise.s_thx = deg2rad(0); params.Qnoise.s_thy = deg2rad(0); params.Qnoise.s_thz = deg2rad(0);
    params.Qnoise.s_wx = deg2rad(0.01); params.Qnoise.s_wy = deg2rad(0.01); params.Qnoise.s_wz = deg2rad(0.01);
    params.bias.tau_gyro = 50; params.bias.tau_acc = 50; params.bias.tau_mag = 1; params.bias.tau_baro = 1;
    params.Qnoise.s_dbgx = 0.01; params.Qnoise.s_dbgy = 0.01; params.Qnoise.s_dbgz = 0.01;
    params.Qnoise.s_dbax = 0.01; params.Qnoise.s_dbay = 0.01; params.Qnoise.s_dbaz = 0.01;
    params.Qnoise.s_dbmx = 0.01; params.Qnoise.s_dbmy = 0.01; params.Qnoise.s_dbmz = 0.01;
    params.Qnoise.s_dbbaro = 0.01;

    %% Fin Parameters (기존 값 유지)
    params.fin.chord = 0.1; % fin chord [m]
    params.fin.span = 0.05; % fin span [m]
    params.fin.num = 4; % fin num
    params.fin.max_angle = deg2rad(10); % max angle [rad]
    params.fin.root_chord = 0.2;    % From image data [m]
    params.fin.tip_chord = 0.06;    % From image data [m]
    params.fin.span_length = 0.13;  % From image data [m]
    params.fin.sweep = 47;          % From image data [deg]

    %% Calculate Aerodynamic Coefficients (기존 계산 로직 유지)
    rocket.length = params.vehicle.L;
    rocket.diameter = params.vehicle.D_ref * 2; % 직경으로 전달해야 할 수 있음
    rocket.nose_length = 0.3;  % 기존 값 유지
    rocket.nose_type = "haack"; % 기존 값 유지
    rocket.cg_position = params.vehicle.L/2;  % 기존 값 유지
    rocket.fin_count = params.fin.num;
    rocket.fin_root_chord = params.fin.root_chord;
    rocket.fin_tip_chord = params.fin.tip_chord;
    rocket.fin_span = params.fin.span_length;
    rocket.fin_sweep = params.fin.sweep;
    % 에러 발생 방지를 위해 calculateRocketCoefficients 함수가 현재 경로에 있거나,
    % 이 함수 내부에 nested function으로 정의되어 있어야 합니다.
    try
        aero_coef = calculateRocketCoefficients(rocket); % 기존 호출 유지
        params.aero.C_A = aero_coef.C_A;
        params.aero.C_S_beta = aero_coef.C_N_alpha; % 명칭 확인 필요
        params.aero.C_N_alpha = aero_coef.C_N_alpha;
        params.aero.C_l_p = aero_coef.C_l_p;
        params.aero.C_m_q = aero_coef.C_m_q;
        params.aero.C_m_alpha = aero_coef.C_m_alpha;
        params.aero.C_n_r = aero_coef.C_n_r;
        params.aero.C_n_beta = aero_coef.C_n_beta;
    catch ME_aero
         fprintf('경고: 공력 계수 계산 중 오류 발생. 공력 파라미터가 불완전할 수 있습니다.\n (%s)\n', ME_aero.message);
         % 필요한 경우 기본값 설정
        params.aero.C_N_alpha = 0; params.aero.C_A = 0; params.aero.C_l_p = 0;
        params.aero.C_m_q = 0; params.aero.C_m_alpha = 0; params.aero.C_n_r = 0; params.aero.C_n_beta = 0;
    end

    %% Wind Parameters (기존 값 유지)
    params.wind.v_avg = 0;
    params.wind.turb_intensity = 0;
    params.wind.direction = 0;

    %% Flight Regime Parameters (기존 값 유지)
    params.aero.mach_crit = 0.8;
    params.aero.mach_super = 1.2;
    params.aero.aoa_crit = deg2rad(17);

    fprintf('기존 파라미터 초기화 완료.\n');

    % =====================================================================
    %   2. XML 파일 읽기 (파라미터 설정은 아직 안 함)
    % =====================================================================
    xmldata = []; % 미리 초기화
    try
        xmldata = xmlread(xmlFilePath);
        fprintf('XML 파일 "%s" 로드 성공.\n', xmlFilePath);
        % --- 여기서 나중에 xmldata를 파싱하여 params 구조체 업데이트 ---
        % 예: params.vehicle.L = extractLengthFromXML(xmldata);
        % 예: [params.motor.thrust_func_stage1, params.motor.mass_func_stage1] = parseMotorFromXML(xmldata);
        % 등등... (지금은 비워둠)
        fprintf('참고: XML 파싱 및 파라미터 업데이트 로직은 아직 구현되지 않았습니다.\n');

    catch ME_xml
        fprintf('XML 파일 "%s" 로드 또는 처리 실패:\n%s\n', xmlFilePath, ME_xml.message);
        fprintf('XML 기반 플로팅을 건너<0xEB><0x9A><0x8D>니다.\n');
        return; % XML 로드 실패 시 플로팅 불가
    end

    % =====================================================================
    %   3. XML 기반 2D 및 3D 플로팅 실행
    % =====================================================================
    n_theta_points_3d = 30; % 3D 플롯 정밀도

    if generate2DPlot
        fprintf('\n--- 2D 프로파일 플로팅 시작 ---\n');
        try
            plot2DProfile(xmldata);
        catch ME_plot2d
            fprintf(2, '2D 플로팅 중 오류 발생: %s\n', ME_plot2d.message);
        end
    end

    if generate3DPlot
        fprintf('\n--- 3D 모델 플로팅 시작 ---\n');
        try
            plot3DProfile(xmldata, n_theta_points_3d);
        catch ME_plot3d
            fprintf(2, '3D 플로팅 중 오류 발생: %s\n', ME_plot3d.message);
        end
    end

    fprintf('\nParams_init_HANul_WithPlotting 함수 실행 완료.\n');


% =========================================================================
%   Nested Functions (헬퍼 함수 + 2D/3D 플로팅 함수)
% =========================================================================

    % --- XML 정보 추출 함수 ---
    function [rocketName, designer] = getRocketInfo(xmldata_in)
        rocketName = 'Unknown Rocket'; designer = 'Unknown';
        rocketNode = xmldata_in.getElementsByTagName('rocket').item(0);
        if isempty(rocketNode), return; end
        rocketName = getNodeValue(rocketNode, 'name', 'Name Not Found');
        designer = getNodeValue(rocketNode, 'designer', 'Unknown');
    end

    function nodeValue = getNodeValue(parentNode, tagName, defaultValue)
        nodeList = parentNode.getElementsByTagName(tagName);
        nodeValue = defaultValue; % 기본값 먼저 설정
        % 노드가 있고, 첫 번째 자식 노드가 있고, 그 자식 노드가 텍스트 노드일 때만 값 읽기
        if nodeList.getLength > 0 && ~isempty(nodeList.item(0)) && ...
           ~isempty(nodeList.item(0).getFirstChild) && ...
           nodeList.item(0).getFirstChild.getNodeType == org.w3c.dom.Node.TEXT_NODE
             nodeValue = strtrim(char(nodeList.item(0).getTextContent));
        elseif nodeList.getLength > 0 && ~isempty(nodeList.item(0)) && ...
               isempty(nodeList.item(0).getFirstChild) % 빈 태그 처리 (예: <designer/>)
             nodeValue = ''; % 빈 문자열 반환
        end
    end


    function subCompNode = findMainComponentListNode(xmldata_in)
        subCompNode = [];
        rocketNode = xmldata_in.getElementsByTagName('rocket').item(0);
        if isempty(rocketNode), fprintf('오류: <rocket> 없음.\n'); return; end
        rocketSubNode = findDirectChildElement(rocketNode, 'subcomponents');
        if isempty(rocketSubNode), fprintf('오류: <rocket> 직속 <subcomponents> 없음.\n'); return; end
        mainStageNode = findDirectChildElement(rocketSubNode, 'stage');
         if isempty(mainStageNode), fprintf('오류: <rocket>/<subcomponents> 직속 <stage> 없음.\n'); return; end
        subCompNode = findDirectChildElement(mainStageNode, 'subcomponents');
         if isempty(subCompNode), fprintf('오류: <stage> 직속 <subcomponents> (부품 목록) 없음.\n'); return; end
        % fprintf('메인 부품 목록 노드 찾기 성공.\n'); % 로그는 내부 함수에서 제거하거나 조절
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
        % getElementsByTagName은 모든 하위 노드를 검색하므로, 직접 자식만 찾으려면
        % findDirectChildElement를 반복 사용하는 로직이 필요할 수 있으나,
        % OpenRocket 구조상 특정 태그는 직접 자식으로만 나타나는 경우가 많아 일단 유지.
        childElements = parentNode.getElementsByTagName(tagName);
    end

    % --- 위치 계산 함수 ---
    function [start_x, position_type_desc] = getAbsoluteStartPosition(componentNode, default_start_x)
        start_x = default_start_x; position_type_desc = 'Sequential';
        posList = getChildElementsByTagName(componentNode, 'position'); % getChildElementsByTagName 사용
        if posList.getLength > 0
            posNode = posList.item(0);
            posValueStr = posNode.getTextContent;
            if ~isempty(posValueStr) % 값이 비어있지 않은지 확인
                 posValue = str2double(posValueStr);
                 posType = char(posNode.getAttribute('type'));
                 if strcmp(posType, 'absolute')
                     start_x = posValue; position_type_desc = sprintf('Absolute (%.4f)', posValue);
                 elseif strcmp(posType, 'bottom') || strcmp(posType, 'top') || strcmp(posType, 'middle')
                     % 이 함수는 절대 시작 위치만 결정하므로, 상대 위치는 Sequential로 간주
                     % (핀 처리 함수에서는 상대 위치 로직 별도 구현)
                     position_type_desc = sprintf('Relative type "%s" - Sequential 적용', posType);
                 else
                     position_type_desc = sprintf('Unknown type "%s" - Sequential 적용', posType);
                 end
            else
                % position 태그는 있으나 값이 비어있는 경우
                position_type_desc = 'Position Tag Empty - Sequential 적용';
            end
        end
    end


    % --- 반경 처리 함수 ---
    function radius = parseRadius(radiusStr, default_radius)
        radius = default_radius; % 기본값 먼저 설정
        if isempty(radiusStr)
             % fprintf('    경고: 반경 문자열 비어있음, 기본값(%.4f) 사용.\n', default_radius); % 로그는 필요시 활성화
             return;
        end
        try
            if startsWith(radiusStr, 'auto')
                parts = strsplit(radiusStr);
                if length(parts) >= 2
                    radius = str2double(parts{2});
                % else % 'auto'만 있는 경우 기본값 유지
                %     fprintf('    경고: ''auto'' 반경 값 없음 ("%s"), 기본값(%.4f) 사용.\n', radiusStr, default_radius);
                end
            else
                radius = str2double(radiusStr);
            end

            % 유효성 검사
            if isnan(radius) || radius < 0 || radius > 10 % 너무 크거나 음수인 반경은 오류로 간주
                 % fprintf('    경고: 비정상/NaN 반경 값(%.4f) 감지, 기본값(%.4f) 사용.\n', radius, default_radius);
                 radius = default_radius;
            end
        catch % str2double 등에서 에러 발생 시
            % fprintf('    경고: 반경 파싱 오류 ("%s"), 기본값(%.4f) 사용.\n', radiusStr, default_radius);
            radius = default_radius;
        end
    end

    % --- 2D 플로팅 함수 (drawRocketProfileFromXML_V2 내용 기반) ---
    function plot2DProfile(xmldata_in)
        % --- 2. 기본 로켓 정보 추출 ---
        [rocketName, ~] = getRocketInfo(xmldata_in);

        % --- 3. 플로팅 준비 ---
        figure('Name', sprintf('2D Profile: %s', rocketName), 'NumberTitle', 'off');
        hold on; axis equal; grid on;
        xlabel('축 방향 위치 (m)'); ylabel('반경 (m)');
        title(sprintf('로켓 2D 프로파일: %s', rocketName));

        % --- 4. 부품 처리 로직 ---
        current_absolute_end_x = 0; current_radius = 0; node_idx = 0;
        componentAbsoluteEndX = []; % 동적 할당 또는 미리 크게 할당

        try
            stageSubcomponentsNode = findMainComponentListNode(xmldata_in);
            if isempty(stageSubcomponentsNode), error('메인 부품 목록 (<stage> 내 <subcomponents>)을 찾을 수 없습니다.'); end
            childNodes = stageSubcomponentsNode.getChildNodes;

            for i = 0:childNodes.getLength-1
                node = childNodes.item(i);
                if node.getNodeType == org.w3c.dom.Node.ELEMENT_NODE
                    node_idx = node_idx + 1;
                    nodeName = char(node.getNodeName);
                    compName = getNodeValue(node, 'name', ['Unnamed ' nodeName]);
                    fprintf('  [2D-%d] 처리: %s (%s)... ', node_idx, compName, nodeName);

                    [absolute_start_x, ~] = getAbsoluteStartPosition(node, current_absolute_end_x);
                    current_x = absolute_start_x;
                    component_len = 0; component_end_radius = current_radius;

                    switch nodeName
                        case 'nosecone'
                            lengthNode = node.getElementsByTagName('length').item(0);
                            aftRadiusNode = node.getElementsByTagName('aftradius').item(0);
                            shapeNode = node.getElementsByTagName('shape').item(0);
                            shapeParamNode = node.getElementsByTagName('shapeparameter').item(0);
                            if ~isempty(lengthNode) && ~isempty(aftRadiusNode) && ~isempty(shapeNode)
                                len = str2double(lengthNode.getTextContent);
                                aftRadius = parseRadius(char(aftRadiusNode.getTextContent), 0.068);
                                shapeType = char(shapeNode.getTextContent);
                                shapeParam = 0; if ~isempty(shapeParamNode), shapeParam = str2double(shapeParamNode.getTextContent); end
                                plotNoseConeShape(shapeType, shapeParam, len, aftRadius, current_x, 'b');
                                component_len = len; component_end_radius = aftRadius;
                            end
                        case 'bodytube'
                            lengthNode = node.getElementsByTagName('length').item(0);
                            radiusNode = node.getElementsByTagName('radius').item(0);
                            if ~isempty(lengthNode) && ~isempty(radiusNode)
                                len = str2double(lengthNode.getTextContent);
                                tube_radius = parseRadius(char(radiusNode.getTextContent), current_radius);
                                x_start = current_x; x_end = current_x + len;
                                plot([x_start, x_end, x_end, x_start, x_start], ...
                                     [tube_radius, tube_radius, -tube_radius, -tube_radius, tube_radius], 'k-', 'LineWidth', 1);
                                component_len = len; component_end_radius = tube_radius;
                                % Fin Set 처리
                                finSetNodes = getChildElementsByTagName(node, 'trapezoidfinset'); % Body Tube 내부 핀셋 검색
                                for j = 0:finSetNodes.getLength-1
                                    processFinSet2D(finSetNodes.item(j), x_start, len, tube_radius); % 2D 핀 처리 함수 호출
                                end
                            end
                        case 'transition'
                            lengthNode = node.getElementsByTagName('length').item(0);
                            foreRadiusNode = node.getElementsByTagName('foreradius').item(0);
                            aftRadiusNode = node.getElementsByTagName('aftradius').item(0);
                            shapeNode = node.getElementsByTagName('shape').item(0);
                             if ~isempty(lengthNode) && ~isempty(foreRadiusNode) && ~isempty(aftRadiusNode) && ~isempty(shapeNode)
                                len = str2double(lengthNode.getTextContent);
                                foreRadius = parseRadius(char(foreRadiusNode.getTextContent), current_radius);
                                aftRadius = parseRadius(char(aftRadiusNode.getTextContent), foreRadius * 0.8);
                                shapeType = char(shapeNode.getTextContent);
                                start_radius_actual = current_radius; % 현재 반경에서 시작
                                x_start = current_x; x_end = current_x + len;
                                plot([x_start, x_end, x_end, x_start, x_start], ...
                                     [start_radius_actual, aftRadius, -aftRadius, -start_radius_actual, start_radius_actual], 'm-', 'LineWidth', 1);
                                component_len = len; component_end_radius = aftRadius;
                            end
                        otherwise % 기타 부품 (길이만 반영하여 위치 계산)
                            lenNode = node.getElementsByTagName('length').item(0);
                            if ~isempty(lenNode), component_len = str2double(lenNode.getTextContent()); else component_len = 0; end
                            component_end_radius = current_radius; % 반경 변화 없음 가정
                    end

                    absolute_end_x = absolute_start_x + component_len;
                    componentAbsoluteEndX(node_idx) = absolute_end_x; % 끝 위치 저장
                    current_absolute_end_x = absolute_end_x;
                    current_radius = component_end_radius;
                    fprintf('완료 (End X: %.4f, End R: %.4f)\n', absolute_end_x, current_radius);
                end % if Element Node
            end % for loop
        catch ME_plot2d_inner
             fprintf(2, '\n*** 2D 처리 중 오류 발생 ***\n');
             fprintf(2, '오류 메시지: %s\n', ME_plot2d_inner.message);
             if ~isempty(ME_plot2d_inner.stack)
                fprintf(2, '오류 위치: %s, %d번째 줄\n', ME_plot2d_inner.stack(1).name, ME_plot2d_inner.stack(1).line);
             end
        end

        hold off; axis tight;
        current_ylim = ylim;
        if ~isempty(current_ylim) && diff(current_ylim) > 1e-6
             ylim_range = max(abs(current_ylim)); % Y축 대칭 고려
             ylim([-ylim_range*1.1, ylim_range*1.1]);
        end
        grid on; title(sprintf('로켓 2D 프로파일: %s ', rocketName));
        fprintf('2D 프로파일 그리기 완료.\n');
    end % plot2DProfile 끝

    % --- 2D 형상 그리기 함수 (plotNoseConeShape) ---
    function plotNoseConeShape(shapeType, shapeParam, L, R, start_x, color)
        n_points = 100; x_local = linspace(0, L, n_points); y_local = zeros(1, n_points);
        try
            if L <= 0 || R <= 0, error('Nosecone L/R <= 0'); end
            switch lower(shapeType)
                case 'ogive', rho = (R^2 + L^2) / (2*R); y_local = sqrt(max(0, rho^2 - (L - x_local).^2)) + R - rho;
                case 'conical', y_local = R * (x_local / L);
                case 'elliptical', y_local = R * sqrt(max(0, 1 - ((x_local-L)/L).^2));
                case 'power', n_pow = shapeParam; if n_pow <= 0 || n_pow > 1, n_pow = 0.5; end; y_local = R * (x_local / L).^n_pow;
                case 'haack', theta = acos(max(-1, min(1, 1 - 2*x_local/L))); y_local = (R / sqrt(pi)) * sqrt(max(0, theta - 0.5*sin(2*theta)));
                otherwise, y_local = R * (x_local / L); color = [color, ':']; % Conical 근사
            end
            y_local(1) = 0; y_local(end) = R; y_local(isnan(y_local) | ~isreal(y_local)) = 0; y_local = max(0, y_local);
        catch ME_nose, fprintf('      오류: 노즈콘 계산. Conical 대체. (%s)\n', ME_nose.message); y_local = R * (x_local / L); color = [color, ':']; y_local(1) = 0; y_local(end) = R; end
        x_abs = start_x + x_local;
        plot(x_abs, y_local, '-', 'Color', color, 'LineWidth', 1.0); % 라인 두께 조절
        plot(x_abs, -y_local, '-', 'Color', color, 'LineWidth', 1.0);
        plot([start_x + L, start_x + L], [-R, R], '-', 'Color', color, 'LineWidth', 1.0); % 끝 라인
    end

    % --- 2D 핀 처리 함수 (processFinSet2D) ---
    function processFinSet2D(finSetNode, parent_abs_start_x, parent_len, parent_radius)
        finName = getNodeValue(finSetNode, 'name', 'Unnamed Fins');
        % 핀 치수 읽기
        rootChord = str2double(getNodeValue(finSetNode, 'rootchord', '0'));
        tipChord = str2double(getNodeValue(finSetNode, 'tipchord', '0'));
        finHeight = str2double(getNodeValue(finSetNode, 'height', '0'));
        sweepLength = str2double(getNodeValue(finSetNode, 'sweeplength', '0'));

        if rootChord <= 0 || finHeight <= 0, return; end % 그릴 수 없음

        % 위치 계산 (TE 기준)
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
        end % position 태그 없으면 TE를 부모 시작점에 (기존 로직 유지)

        % 핀 꼭지점 좌표 계산 (항상 TE 기준)
        x2 = ref_coord_x; x1 = x2 - rootChord; x4 = x1 + sweepLength; x3 = x4 + tipChord;
        y1 = parent_radius; y2 = parent_radius; y3 = parent_radius + finHeight; y4 = parent_radius + finHeight;

        fin_x = [x1, x2, x3, x4, x1];
        fin_y_top = [y1, y2, y3, y4, y1];
        fin_y_bottom = [-y1, -y2, -y3, -y4, -y1];

        plot(fin_x, fin_y_top, 'r-', 'LineWidth', 0.8); % 라인 두께 조절
        plot(fin_x, fin_y_bottom, 'r-', 'LineWidth', 0.8);
    end

    % --- 3D 플로팅 함수 (drawRocketProfile3D 내용 기반) ---
    function plot3DProfile(xmldata_in, n_theta)
        % --- 2. 기본 로켓 정보 추출 ---
        [rocketName, ~] = getRocketInfo(xmldata_in);

        % --- 3. 3D 플로팅 준비 ---
        figure('Name', sprintf('3D Model: %s', rocketName), 'NumberTitle', 'off');
        hold on; axis equal; grid on;
        xlabel('X (Axial, m)'); ylabel('Y (m)'); zlabel('Z (m)');
        title(sprintf('로켓 3D 모델: %s', rocketName));
        view(3); rotate3d on;

        % --- 4. 부품 처리 로직 ---
        current_absolute_end_x = 0; current_radius = 0; node_idx = 0;

        try
            stageSubcomponentsNode = findMainComponentListNode(xmldata_in);
            if isempty(stageSubcomponentsNode), error('메인 부품 목록 (<stage> 내 <subcomponents>)을 찾을 수 없습니다.'); end
            childNodes = stageSubcomponentsNode.getChildNodes;

            for i = 0:childNodes.getLength-1
                node = childNodes.item(i);
                if node.getNodeType == org.w3c.dom.Node.ELEMENT_NODE
                     node_idx = node_idx + 1;
                     nodeName = char(node.getNodeName);
                     compName = getNodeValue(node, 'name', ['Unnamed ' nodeName]);
                     fprintf('  [3D-%d] 처리: %s (%s)... ', node_idx, compName, nodeName);

                     [absolute_start_x, ~] = getAbsoluteStartPosition(node, current_absolute_end_x);
                     current_x = absolute_start_x;
                     component_len = 0; component_end_radius = current_radius;
                     component_color = [0.7 0.7 0.7]; % 기본 회색

                    switch nodeName
                        case 'nosecone'
                            component_color = 'blue';
                            lengthNode = node.getElementsByTagName('length').item(0);
                            aftRadiusNode = node.getElementsByTagName('aftradius').item(0);
                            shapeNode = node.getElementsByTagName('shape').item(0);
                            shapeParamNode = node.getElementsByTagName('shapeparameter').item(0);
                             if ~isempty(lengthNode) && ~isempty(aftRadiusNode) && ~isempty(shapeNode)
                                len = str2double(lengthNode.getTextContent);
                                aftRadius = parseRadius(char(aftRadiusNode.getTextContent), 0.068);
                                shapeType = char(shapeNode.getTextContent);
                                shapeParam = 0; if ~isempty(shapeParamNode), shapeParam = str2double(shapeParamNode.getTextContent); end
                                plotNoseCone3D(shapeType, shapeParam, len, aftRadius, current_x, component_color, n_theta);
                                component_len = len; component_end_radius = aftRadius;
                            end
                        case 'bodytube'
                            component_color = [0.5 0.5 0.5]; % 약간 더 어두운 회색
                            lengthNode = node.getElementsByTagName('length').item(0);
                            radiusNode = node.getElementsByTagName('radius').item(0);
                            if ~isempty(lengthNode) && ~isempty(radiusNode)
                                len = str2double(lengthNode.getTextContent);
                                tube_radius = parseRadius(char(radiusNode.getTextContent), current_radius);
                                plotBodyTube3D(current_x, len, tube_radius, component_color, n_theta);
                                component_len = len; component_end_radius = tube_radius;
                                % Fin Set 처리
                                finSetNodes = getChildElementsByTagName(node, 'trapezoidfinset');
                                finCount = 4; % 기본값, 필요시 fincount 태그 읽기
                                % finCountNode = findDirectChildElement(node, 'fincount'); % 이 구조는 아닐 수 있음
                                finCountFromSet = getNodeValue(node, 'fincount', '4'); % finset 자체에서 읽기 시도
                                try finCount = str2double(finCountFromSet); catch, finCount = 4; end
                                if isnan(finCount) || finCount <= 0, finCount = 4; end

                                for j = 0:finSetNodes.getLength-1
                                    processFinSet3D(finSetNodes.item(j), current_x, len, tube_radius, finCount, n_theta);
                                end
                            end
                        case 'transition'
                            component_color = 'magenta';
                            lengthNode = node.getElementsByTagName('length').item(0);
                            foreRadiusNode = node.getElementsByTagName('foreradius').item(0);
                            aftRadiusNode = node.getElementsByTagName('aftradius').item(0);
                            shapeNode = node.getElementsByTagName('shape').item(0);
                             if ~isempty(lengthNode) && ~isempty(foreRadiusNode) && ~isempty(aftRadiusNode) && ~isempty(shapeNode)
                                len = str2double(lengthNode.getTextContent);
                                foreRadius = parseRadius(char(foreRadiusNode.getTextContent), current_radius);
                                aftRadius = parseRadius(char(aftRadiusNode.getTextContent), foreRadius * 0.8);
                                % shapeType = char(shapeNode.getTextContent); % Conical로 가정
                                start_radius_actual = current_radius;
                                plotTransition3D(current_x, len, start_radius_actual, aftRadius, component_color, n_theta);
                                component_len = len; component_end_radius = aftRadius;
                            end
                        otherwise
                            lenNode = node.getElementsByTagName('length').item(0);
                            if ~isempty(lenNode), component_len = str2double(lenNode.getTextContent()); else component_len = 0; end
                            component_end_radius = current_radius;
                    end

                    absolute_end_x = absolute_start_x + component_len;
                    current_absolute_end_x = absolute_end_x;
                    current_radius = component_end_radius;
                     fprintf('완료 (End X: %.4f, End R: %.4f)\n', absolute_end_x, current_radius);
                end % if Element Node
            end % for loop
        catch ME_plot3d_inner
             fprintf(2, '\n*** 3D 처리 중 오류 발생 ***\n');
             fprintf(2, '오류 메시지: %s\n', ME_plot3d_inner.message);
             if ~isempty(ME_plot3d_inner.stack)
                fprintf(2, '오류 위치: %s, %d번째 줄\n', ME_plot3d_inner.stack(1).name, ME_plot3d_inner.stack(1).line);
             end
        end

        hold off; axis tight; daspect([1 1 1]);
        camlight left; lighting gouraud; material dull;
        fprintf('3D 모델 그리기 완료.\n');
    end % plot3DProfile 끝

    % --- 3D 형상 그리기 함수 (Nose, Body, Transition, Fin) ---
    function plotNoseCone3D(shapeType, shapeParam, L, R, start_x, color, n_theta)
        n_points = 50; x_local = linspace(0, L, n_points); y_local = zeros(1, n_points);
        try % 계산 로직은 2D와 동일
            if L <= 0 || R <= 0, error('Nosecone L/R 오류'); end
             switch lower(shapeType)
                 case 'ogive', rho = (R^2 + L^2) / (2*R); y_local = sqrt(max(0, rho^2 - (L - x_local).^2)) + R - rho;
                 case 'conical', y_local = R * (x_local / L);
                 case 'elliptical', y_local = R * sqrt(max(0, 1 - ((x_local-L)/L).^2));
                 case 'power', n_pow = shapeParam; if n_pow <= 0 || n_pow > 1, n_pow = 0.5; end; y_local = R * (x_local / L).^n_pow;
                 case 'haack', theta = acos(max(-1, min(1, 1 - 2*x_local/L))); y_local = (R / sqrt(pi)) * sqrt(max(0, theta - 0.5*sin(2*theta)));
                 otherwise, y_local = R * (x_local / L); % Conical 가정
             end
             y_local(1) = 0; y_local(end) = R; y_local(isnan(y_local) | ~isreal(y_local)) = 0; y_local = max(0, y_local);
        catch ME_nose3d, fprintf(' 3D 노즈콘 계산 실패. (%s)\n', ME_nose3d.message); return; end
        theta = linspace(0, 2*pi, n_theta);
        X_surf = start_x + repmat(x_local(:), 1, n_theta); Y_surf = y_local(:) * cos(theta); Z_surf = y_local(:) * sin(theta);
        surf(X_surf, Y_surf, Z_surf, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 0.95);
    end

    function plotBodyTube3D(start_x, len, radius, color, n_theta)
        if len <= 0 || radius <= 0, return; end
        [X_unit, Y_unit, Z_unit] = cylinder(1, n_theta - 1);
        X_surf = start_x + Z_unit * len; Y_surf = Y_unit * radius; Z_surf = X_unit * radius;
        surf(X_surf, Y_surf, Z_surf, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 0.95);
    end

    function plotTransition3D(start_x, len, start_radius, end_radius, color, n_theta)
        if len <= 0, return; end
        x_local = [0, len]; r_profile = [start_radius, end_radius];
        theta = linspace(0, 2*pi, n_theta);
        X_surf = start_x + repmat(x_local(:), 1, n_theta); Y_surf = r_profile(:) * cos(theta); Z_surf = r_profile(:) * sin(theta);
        surf(X_surf, Y_surf, Z_surf, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 0.95);
    end

    function processFinSet3D(finSetNode, parent_abs_start_x, parent_len, parent_radius, fin_count, n_theta)
        % finName = getNodeValue(finSetNode, 'name', 'Unnamed Fins'); % 이름은 현재 미사용
        rootChord = str2double(getNodeValue(finSetNode, 'rootchord', '0'));
        tipChord = str2double(getNodeValue(finSetNode, 'tipchord', '0'));
        finHeight = str2double(getNodeValue(finSetNode, 'height', '0'));
        sweepLength = str2double(getNodeValue(finSetNode, 'sweeplength', '0'));
        if rootChord <= 0 || finHeight <= 0, return; end

        % 위치 계산 (TE 기준)
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
         end

        % 핀 꼭지점 좌표 계산 (TE 기준)
        x2 = ref_coord_x; x1 = x2 - rootChord; x4 = x1 + sweepLength; x3 = x4 + tipChord;
        y1 = parent_radius; y2 = parent_radius; y3 = parent_radius + finHeight; y4 = parent_radius + finHeight;
        verts_2d = [x1 y1; x2 y2; x3 y3; x4 y4];

        % 핀 개수만큼 회전하며 그리기
        fin_angles = linspace(0, 2*pi, fin_count + 1); fin_angles = fin_angles(1:fin_count);
        for i = 1:fin_count
            phi = fin_angles(i);
            verts_3d_rotated = zeros(4, 3);
            verts_3d_rotated(:,1) = verts_2d(:,1); % X
            verts_3d_rotated(:,2) = verts_2d(:,2) .* cos(phi); % Y
            verts_3d_rotated(:,3) = verts_2d(:,2) .* sin(phi); % Z
            patch('Vertices', verts_3d_rotated, 'Faces', [1 2 3 4], ...
                  'FaceColor', 'red', 'EdgeColor', [0.3 0 0], 'FaceAlpha', 0.9); % 약간 어두운 빨강 테두리
        end
    end

    % --- calculateRocketCoefficients 함수 ---
    % 이 함수는 외부에 정의되어 있거나, 여기에 nested function으로 포함되어야 합니다.
    % 임시로 여기에 정의합니다 (기존 코드에서 가져옴).
    function coef = calculateRocketCoefficients(rocket)
        % Barrowman 방정식 기반 공력 계수 계산 (기존 코드)
        S_ref = pi * (rocket.diameter/2)^2; CNa_nose = 2.4; % haack 가정
        CNa_body = 2 * (rocket.length - rocket.nose_length) * rocket.diameter / S_ref;
        Af = (rocket.fin_root_chord + rocket.fin_tip_chord) * rocket.fin_span / 2; AR = 2 * rocket.fin_span^2 / Af;
        Kfb = 1 + rocket.diameter/(2 * rocket.fin_span);
        CNa_fins = Kfb * (4 * rocket.fin_count * (AR / (2 + sqrt(AR^2 + 4)))) * (Af/S_ref);
        CNa_total = CNa_nose + CNa_body + CNa_fins;
        Xcp_nose = 0.466 * rocket.nose_length; Xcp_body = rocket.nose_length + (rocket.length - rocket.nose_length)/2;
        MAC = (rocket.fin_root_chord + rocket.fin_tip_chord)/2; Xcp_fin = rocket.length - MAC/4; % 오류 가능성: 부품 위치 고려 안됨
        Xcp = (CNa_nose * Xcp_nose + CNa_body * Xcp_body + CNa_fins * Xcp_fin) / CNa_total; % 단순화된 계산
        coef.C_N_alpha = CNa_total; coef.C_A = 0.1 + 0.1 * (rocket.length/rocket.diameter); % 단순화
        coef.C_l_p = -rocket.fin_count * CNa_fins * rocket.fin_span^2 / (4 * S_ref * rocket.length); % 단순화
        coef.C_m_q = -2 * (Xcp - rocket.cg_position) * CNa_total / rocket.length; coef.C_n_r = coef.C_m_q;
        coef.C_m_alpha = -(Xcp - rocket.cg_position) * CNa_total / rocket.length; coef.C_n_beta = coef.C_m_alpha;
        coef.Xcp = Xcp; coef.stability_margin = (Xcp - rocket.cg_position) / rocket.diameter;
    end

end