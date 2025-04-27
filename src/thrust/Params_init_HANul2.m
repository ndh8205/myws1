function params = Params_init_HANul2()
    xmlFilePath = 'rocket.xml'; % OpenRocket XML 파일 경로 지정

    extractedParams = extractParamsFromORK_Strict(xmlFilePath);

    % XML 읽기 실패 시 기본값 사용 또는 오류 처리
    if isempty(fieldnames(extractedParams))
        warning('XML 파라미터 읽기 실패! 하드코딩된 기본값을 사용합니다.');
        % 여기에 기본값 설정 로직 추가 가능
        use_default_geometry = true;
    else
        use_default_geometry = false;
        fprintf('XML 파라미터 로드 완료.\n');
    end

    params.environment.g = [0, 0, 9.81]'; % [m/s^2]
    params.environment.ro = 1.666; % [kg/m^3] - Need verification
    params.environment.ro_2 = 1.24; % [kg/m^3] - Need verification
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

    params.control.Ka_pp = [0, 0, 0]'; % Attitude P
    params.control.Ka_p = [0, 0, 0]'; % Rate P
    params.control.Ka_i = [0, 0, 0]'; % Rate I
    params.control.Ka_d = [0, 0, 0]'; % Rate D

    % 질량 및 관성 모멘트는 여전히 하드코딩 필요 (XML에서 직접 추출 불가)
    params.vehicle.J_X = 0.061; % [kgm^2] - TODO: Update based on design
    params.vehicle.J_Y = 13.4;  % [kgm^2] - TODO: Update based on design
    params.vehicle.J_Z = 13.4;  % [kgm^2] - TODO: Update based on design
    params.vehicle.m_D = 20.014; % [kg] (Dry mass) - TODO: Update based on design
    params.vehicle.m_W_const = 23.647; % [kg] (Wet mass) - TODO: Update based on design

    if ~use_default_geometry && isfield(extractedParams, 'vehicle')
        params.vehicle.L = extractedParams.vehicle.L;         % [m] Rocket length from XML
        params.vehicle.D_ref = extractedParams.vehicle.D_ref; % [m] Rocket diameter from XML
        params.vehicle.r_ref = extractedParams.vehicle.r_ref; % [m] Rocket radius from XML
        fprintf('Updated Vehicle Geometry from XML: L=%.3f, D=%.3f\n', params.vehicle.L, params.vehicle.D_ref);
    else
        params.vehicle.L = 2.71;    % [m] Default Rocket length
        params.vehicle.D_ref = 0.13; % [m] Default Rocket diameter
        params.vehicle.r_ref = 0.065;% [m] Default Rocket radius
        fprintf('Using Default Vehicle Geometry: L=%.3f, D=%.3f\n', params.vehicle.L, params.vehicle.D_ref);
    end

    params.vehicle.S_A_ref = pi * params.vehicle.r_ref^2; % [m^2] Cross-sectional area (calculated)
    params.vehicle.S_W_ref = 1.1068; % [m^2] Wetted surface area - TODO: Estimate or update

    params.vehicle.Lp = -0.32; % [m] Center of Pressure distance - TODO: Update (calculated later)
    params.vehicle.Lt = -1.06; % [m] Center to Nozzle distance - TODO: Update based on design
    params.vehicle.Lrcs = 1;   % [m] Center to RCS distance - TODO: Update based on design
    params.vehicle.RCS_T = 1;  % [N] RCS thruster force - TODO: Update based on design

    params.vehicle.J = [params.vehicle.J_X, 0, 0;
                       0, params.vehicle.J_Y, 0;
                       0, 0, params.vehicle.J_Z];

    opts = detectImportOptions('AeroTech_M2400T.csv', 'VariableNamingRule', 'preserve');
    data_stage1 = readtable('AeroTech_M2400T.csv', opts);
    params.motor.E1M_d = data_stage1.("Mass(g)")(end)/1000;
    time_stage1 = data_stage1.("Time(s)");
    total_time = max(time_stage1) - min(time_stage1);
    sampling_rate = 400;
    interp_point_1 = round(total_time * sampling_rate) + 1;
    new_time_stage1 = linspace(min(time_stage1), max(time_stage1), interp_point_1);
    interp_mass_stage1 = interp1(time_stage1, data_stage1.("Mass(g)")/1000, new_time_stage1, 'linear');
    interp_thrust_stage1 = interp1(time_stage1, data_stage1.("Thrust(N)"), new_time_stage1, 'linear');
    params.motor.thrust_func_stage1 = interp_thrust_stage1;
    params.motor.mass_func_stage1 = interp_mass_stage1;
    params.vehicle.m_W = params.motor.mass_func_stage1(1); % Set initial wet mass based on motor data start
    params.motor.Ti_1 = interp_point_1;

    params.Qnoise.s_px = 0; params.Qnoise.s_py = 0; params.Qnoise.s_pz = 0;
    params.Qnoise.s_u = 0.01; params.Qnoise.s_v = 0.01; params.Qnoise.s_w = 0.01;
    params.Qnoise.s_thx = deg2rad(0); params.Qnoise.s_thy = deg2rad(0); params.Qnoise.s_thz = deg2rad(0);
    params.Qnoise.s_wx = deg2rad(0.01); params.Qnoise.s_wy = deg2rad(0.01); params.Qnoise.s_wz = deg2rad(0.01);
    params.bias.tau_gyro = 50; params.bias.tau_acc = 50; params.bias.tau_mag = 1; params.bias.tau_baro = 1;
    params.Qnoise.s_dbgx = 0.01; params.Qnoise.s_dbgy = 0.01; params.Qnoise.s_dbgz = 0.01;
    params.Qnoise.s_dbax = 0.01; params.Qnoise.s_dbay = 0.01; params.Qnoise.s_dbaz = 0.01;
    params.Qnoise.s_dbmx = 0.01; params.Qnoise.s_dbmy = 0.01; params.Qnoise.s_dbmz = 0.01;
    params.Qnoise.s_dbbaro = 0.01;

    if ~use_default_geometry && isfield(extractedParams, 'fin') && ~isempty(fieldnames(extractedParams.fin))
        params.fin.num = extractedParams.fin.num;
        params.fin.root_chord = extractedParams.fin.root_chord;
        params.fin.tip_chord = extractedParams.fin.tip_chord;
        params.fin.span_length = extractedParams.fin.span_length; % OpenRocket 'height' is span
        params.fin.sweep = extractedParams.fin.sweep_angle; % Use calculated angle
        % params.fin.chord = (params.fin.root_chord + params.fin.tip_chord) / 2; % Calculate average chord if needed
        % params.fin.span = params.fin.span_length; % Use span_length for consistency
        fprintf('Updated Fin Geometry from XML: Num=%d, RootC=%.3f, TipC=%.3f, Span=%.3f, Sweep=%.1f deg\n', ...
                params.fin.num, params.fin.root_chord, params.fin.tip_chord, params.fin.span_length, params.fin.sweep);
    else
        params.fin.num = 4;
        params.fin.root_chord = 0.2;
        params.fin.tip_chord = 0.06;
        params.fin.span_length = 0.13;
        params.fin.sweep = 47; % [deg]
        fprintf('Using Default Fin Geometry.\n');
    end
    params.fin.max_angle = deg2rad(10); % max angle [rad] - Keep as design parameter

    % Define rocket geometry structure for coefficient calculation
    rocket = struct();
    rocket.length = params.vehicle.L;
    rocket.diameter = params.vehicle.D_ref; % Use diameter directly

    % Use Nosecone info from XML if available
    if ~use_default_geometry && isfield(extractedParams, 'nosecone')
        rocket.nose_length = extractedParams.nosecone.length;
        rocket.nose_type = extractedParams.nosecone.type; % e.g., "ogive", "conical"
    else
        rocket.nose_length = 0.3;  % Default nose cone length
        rocket.nose_type = "haack"; % Default nose cone type
    end

    % CG position - CRITICAL: This needs to be determined accurately for the specific design.
    % Using L/2 is a rough approximation and likely incorrect.
    % TODO: Calculate or obtain CG position accurately!
    rocket.cg_position = params.vehicle.L / 2;
    fprintf('Warning: Using approximate CG position (L/2 = %.3f m). Update required!\n', rocket.cg_position);

    % Fin parameters from params.fin (already updated from XML or default)
    rocket.fin_count = params.fin.num;
    rocket.fin_root_chord = params.fin.root_chord;
    rocket.fin_tip_chord = params.fin.tip_chord;
    rocket.fin_span = params.fin.span_length;
    rocket.fin_sweep = params.fin.sweep; % Use sweep angle in degrees

    % Calculate coefficients using Barrowman equations
    aero_coef = calculateRocketCoefficients(rocket); % Pass the structure

    % Store calculated coefficients
    params.aero.C_A = aero_coef.C_A;
    params.aero.C_S_beta = aero_coef.C_N_alpha; % Assuming symmetric rocket C_N_beta = C_N_alpha
    params.aero.C_N_alpha = aero_coef.C_N_alpha;
    params.aero.C_l_p = aero_coef.C_l_p;
    params.aero.C_m_q = aero_coef.C_m_q;
    params.aero.C_m_alpha = aero_coef.C_m_alpha;
    params.aero.C_n_r = aero_coef.C_n_r;
    params.aero.C_n_beta = aero_coef.C_n_beta;

    % Update Center of Pressure based on calculation
    params.vehicle.Lp = rocket.cg_position - aero_coef.Xcp; % Lp is distance from CG to CP (negative if CP is behind CG)
    fprintf('Calculated Aero Coefficients and CP:\n');
    fprintf('  C_N_alpha: %.3f\n', params.aero.C_N_alpha);
    fprintf('  Xcp (from nose tip): %.3f m\n', aero_coef.Xcp);
    fprintf('  Lp (CG to CP): %.3f m\n', params.vehicle.Lp);
    fprintf('  Stability Margin (calibers): %.2f\n', aero_coef.stability_margin);


    params.wind.v_avg = 0;           % Average wind speed [m/s]
    params.wind.turb_intensity = 0; % Turbulence intensity
    params.wind.direction = 0;        % Wind direction [rad]

    params.aero.mach_crit = 0.8;     % Critical Mach number
    params.aero.mach_super = 1.2;    % Supersonic Mach number
    params.aero.aoa_crit = deg2rad(17); % Critical angle of attack [rad]

    fprintf('Parameter initialization complete.\n');

end

% =========================================================================
% Aerodynamic Coefficient Calculation Function (Nested or Separate File)
% =========================================================================
function coef = calculateRocketCoefficients(rocket)
    % Calculate rocket aerodynamic coefficients based on Barrowman equations (Subsonic)
    % Input: rocket - structure containing geometry (length, diameter, nose_*, fin_*)

    fprintf('Calculating aerodynamic coefficients...\n');
    % Reference dimensions
    S_ref = pi * (rocket.diameter/2)^2; % Cross-sectional area
    d = rocket.diameter;

    % --- 1. Nose Cone Contribution ---
    Ln = rocket.nose_length;
    switch lower(rocket.nose_type)
        case {"ogive", "elliptical", "power"} % Approximate Ogive/Elliptical/Power as similar for CNa
            CNa_nose = 2.0;
        case "conical"
            CNa_nose = 2.0;
        case "haack"
             CNa_nose = 2.0; % Haack series CNa is close to 2
        otherwise
            fprintf('  Warning: Unknown nose type "%s". Assuming CNa_nose = 2.0.\n', rocket.nose_type);
            CNa_nose = 2.0;
    end
    % Center of Pressure for Nose Cone (Xcp measured from nose tip)
    switch lower(rocket.nose_type)
        case "ogive"
            Xcp_nose = 0.466 * Ln; % Approximation for tangent ogive
        case "conical"
            Xcp_nose = 0.666 * Ln;
        case "elliptical"
            Xcp_nose = 0.666 * Ln; % Approximation
        case "power"
             Xcp_nose = 0.75 * Ln; % Approximation for n=0.5 power series
        case "haack"
             Xcp_nose = 0.5 * Ln; % Approximation for LV-Haack
        otherwise
            Xcp_nose = 0.5 * Ln; % Default approximation
    end
    fprintf('  Nose: Type=%s, CNa=%.2f, Xcp=%.3f\n', rocket.nose_type, CNa_nose, Xcp_nose);


    % --- 2. Body Tube Contribution (excluding nose cone) ---
    % Note: Barrowman often ignores body lift contribution, but it can be added.
    % Simplified approach: Ignore body lift contribution to CNa for stability calcs,
    % as it's small and acts near the body's geometric center.
    CNa_body = 0;
    Xcp_body = Ln + (rocket.length - Ln) / 2; % Geometric center of the cylindrical part


    % --- 3. Fin Contribution ---
    N = rocket.fin_count;           % Number of fins
    Cr = rocket.fin_root_chord;
    Ct = rocket.fin_tip_chord;
    s = rocket.fin_span;            % Fin span (height)
    Lf = sqrt(rocket.fin_sweep^2 + s^2); % Length of leading edge (approx) - sweep is angle here
    sweep_rad = deg2rad(rocket.fin_sweep); % Sweep angle in radians
    sweep_LE_deg = atan( tan(sweep_rad) + (Cr-Ct)/(2*s) ) * 180/pi; % Leading edge sweep angle [deg]

    % Aspect Ratio (Geometric)
    Af_planform = (Cr + Ct) * s / 2; % Planform area of one fin
    AR = (2*s)^2 / (Af_planform * 2); % Aspect ratio based on total span (2s) and total area (2*Af) - Check definition

    % Interference factor (Body-on-Fin and Fin-on-Body)
    K_fb = 1 + (rocket.diameter/2) / (s + rocket.diameter/2);

    % CNa for a single fin panel (approximation using Polhamus leading-edge suction analogy for low aspect ratio)
    % Or simpler DATCOM method for subsonic:
    beta_mach = 1; % Assuming subsonic (Mach approx 0)
    CNa_fin_panel = (2 * pi * AR) / (2 + sqrt(4 + (AR*beta_mach)^2 * (1 + tan(deg2rad(sweep_LE_deg))^2 / beta_mach^2)));

    % Total CNa for the fin set
    CNa_fins = K_fb * CNa_fin_panel * (N/2); % N/2 because CNa is per radian, often based on planform area S_ref

    % Center of Pressure for Fins (Xcp measured from nose tip)
    % Position of fin TE relative to nose tip (approximate - needs accurate fin position)
    % Assuming fin TE is at rocket end for this simplified example:
    X_fin_TE_root = rocket.length; % Approximation - TODO: Use actual fin position from XML
    X_fin_LE_root = X_fin_TE_root - Cr;

    % Xcp of the fin set relative to the fin root leading edge
    Xcp_fin_local = (Cr * (Cr + 2*Ct) + Ct^2) / (3 * (Cr + Ct)) + (s * tan(sweep_rad) * (Cr + 2*Ct)) / (6 * (Cr + Ct)); % Approximation based on trapezoid geometry + sweep
    % Absolute Xcp of the fin set from the nose tip
    Xcp_fins = X_fin_LE_root + Xcp_fin_local;
    fprintf('  Fins: N=%d, AR=%.2f, CNa_panel=%.3f, K_fb=%.2f, CNa_fins=%.3f, Xcp_fins=%.3f\n', N, AR, CNa_fin_panel, K_fb, CNa_fins, Xcp_fins);


    % --- Total Normal Force Coefficient Derivative (CNa) ---
    CNa_total = CNa_nose + CNa_fins; % Ignoring body contribution for simplicity

    % --- Total Center of Pressure (Xcp) ---
    % Weighted average based on CNa contributions (measured from nose tip)
    if abs(CNa_total) > 1e-6
        Xcp = (CNa_nose * Xcp_nose + CNa_fins * Xcp_fins) / CNa_total;
    else
        Xcp = rocket.length / 2; % Default if CNa is zero
    end

    % --- Axial Force Coefficient (CA) ---
    % Very rough estimate - depends heavily on Mach number, Reynolds number, surface finish
    CA_base = 0.05; % Base drag (blunt base assumption)
    CA_skin_friction = 0.003 * (rocket.length / d); % Skin friction estimate (turbulent)
    CA_nose_pressure = 0.05; % Nose pressure drag estimate
    CA_fin_drag = 0.01 * N; % Fin drag estimate
    coef.C_A = CA_base + CA_skin_friction + CA_nose_pressure + CA_fin_drag; % Simplified subsonic estimate

    % --- Damping Coefficients ---
    % Roll Damping (Clp) - Primarily due to fins
    % Rough estimate based on DATCOM methods (per radian/sec)
    % Depends on fin location relative to CG
    X_cg = rocket.cg_position;
    coef.C_l_p = - N * CNa_fin_panel * (s + d/2)^2 / (2 * d^2) * (Xcp_fins - X_cg) / rocket.length; % Highly approximate

    % Pitch/Yaw Damping (Cmq, Cnr) - Due to nose and fins primarily
    % Depends significantly on distance from CG
    coef.C_m_q = -2 * CNa_total * ((Xcp - X_cg)/d)^2; % Simplified estimate based on total CNa and static margin squared
    coef.C_n_r = coef.C_m_q; % Assume symmetric rocket

    % --- Static Stability Derivatives ---
    % Pitch/Yaw Moment Coefficient Slope (Cm_alpha, Cn_beta)
    coef.C_m_alpha = CNa_total * (X_cg - Xcp) / d; % Per radian, based on static margin
    coef.C_n_beta = -coef.C_m_alpha; % Cn_beta = -Cm_alpha for standard aero axes

    % --- Store results ---
    coef.C_N_alpha = CNa_total; % [/rad]
    coef.Xcp = Xcp; % [m] from nose tip
    coef.stability_margin = (Xcp - X_cg) / d; % [calibers] Static margin

    fprintf('  Calculated: CNa_total=%.3f, Xcp=%.3f, Stability Margin=%.2f cal\n', CNa_total, Xcp, coef.stability_margin);
end
function rocketParams = extractParamsFromORK_Strict(xmlFilePath)
    % Extracts parameters strictly following the logic of the provided ork2matlab3d code.
    % 입력: xmlFilePath - OpenRocket XML 파일 경로
    % 출력: rocketParams - 추출된 기하학적 파라미터를 담은 구조체

    fprintf('Extracting parameters from: %s (Strict Mode)\n', xmlFilePath);
    rocketParams = struct(); % Initialize output structure

    % --- 1. XML 파일 읽기 ---
    try
        xmldata = xmlread(xmlFilePath);
        fprintf('XML file loaded successfully.\n');
    catch ME
        error('Failed to load XML file "%s":\n%s', xmlFilePath, ME.message);
    end

    % --- 2. 기본 로켓 정보 추출 ---
    [rocketName, designer] = getRocketInfo_Strict(xmldata); % Use strict helper
    rocketParams.name = rocketName;
    fprintf('Rocket Name: %s\n', rocketName);
    fprintf('Designer: %s\n\n', designer);

    % --- 3. 부품 처리 로직 (파라미터 추출 전용) ---
    current_absolute_end_x = 0;
    current_radius = 0;
    max_radius = 0;
    fin_sets_data = {}; % Store extracted fin set data
    all_component_end_x = [];

    try
        stageSubcomponentsNode = findMainComponentListNode_Strict(xmldata); % Use strict helper
        if isempty(stageSubcomponentsNode)
            error('Could not find the main component list.');
        end

        fprintf('\n--- Component Processing (Parameter Extraction Only) ---\n');
        childNodes = stageSubcomponentsNode.getChildNodes;
        fprintf('Nodes to process in main <subcomponents>: %d\n', childNodes.getLength);

        node_idx = 0;

        for i = 0:childNodes.getLength-1
            node = childNodes.item(i);
            nodeType = -1;
            % Basic check for node validity before getting type
            if ~isempty(node) && ismethod(node, 'getNodeType')
                 try nodeType = node.getNodeType(); catch, nodeType = -1; end
            end

            if nodeType == org.w3c.dom.Node.ELEMENT_NODE
                node_idx = node_idx + 1;
                nodeName = char(node.getNodeName);
                compName = getNodeValue_Strict(node, 'name', ['Unnamed ' nodeName]); % Use strict helper
                fprintf('\n>> [%d] Processing: %s (%s)\n', node_idx, compName, nodeName);

                % --- 3.1. 위치 결정 ---
                [absolute_start_x, position_type_desc] = getAbsoluteStartPosition_Strict(node, current_absolute_end_x); % Use strict helper
                fprintf('    Absolute Start X: %.4f m (Method: %s)\n', absolute_start_x, position_type_desc);
                current_x = absolute_start_x;

                % --- 3.2. 타입별 파라미터 추출 ---
                component_len = 0;
                component_end_radius = current_radius;

                switch nodeName
                    case 'nosecone'
                        len = str2double(getNodeValue_Strict(node, 'length', '0'));
                        aftRadiusStr = getNodeValue_Strict(node, 'aftradius', 'auto');
                        shapeType = getNodeValue_Strict(node, 'shape', 'ogive');
                        shapeParam = str2double(getNodeValue_Strict(node, 'shapeparameter', '0'));
                        aftRadius = parseRadius_Strict(aftRadiusStr, 0.068); % Use strict helper (default from ork3d)

                        fprintf('    Nosecone Params: L=%.3f, R_aft=%.3f, Type=%s, Param=%.2f\n', len, aftRadius, shapeType, shapeParam);

                        % 파라미터 저장
                        rocketParams.nosecone.length = len;
                        rocketParams.nosecone.aft_radius = aftRadius;
                        rocketParams.nosecone.type = shapeType;
                        rocketParams.nosecone.shape_param = shapeParam;

                        component_len = len;
                        component_end_radius = aftRadius;

                    case 'bodytube'
                        len = str2double(getNodeValue_Strict(node, 'length', '0'));
                        radiusStr = getNodeValue_Strict(node, 'radius', 'auto');
                        tube_radius = parseRadius_Strict(radiusStr, current_radius); % Use strict helper

                        fprintf('    Bodytube Params: L=%.3f, Radius=%.3f\n', len, tube_radius);
                        if abs(tube_radius - current_radius) > 1e-6 && current_radius ~= 0
                            fprintf('      Radius mismatch warning (%.4f -> %.4f).\n', current_radius, tube_radius);
                        end

                        component_len = len;
                        component_end_radius = tube_radius;

                        % Fin Set 파라미터 추출
                        finSetNodes = getChildElementsByTagName_Strict(node, 'trapezoidfinset'); % Use strict helper
                        finListLength = 0; if ~isempty(finSetNodes), try finListLength = finSetNodes.getLength(); catch, finListLength=0; end, end
                        fprintf('    Found %d fin set(s) inside.\n', finListLength);

                        if finListLength > 0
                            for j = 0:finListLength-1
                                finNode = finSetNodes.item(j);
                                if ~isempty(finNode)
                                    % ork2matlab3d의 processFinSet3D 로직을 따라 파라미터 추출
                                    finData = extractFinData_Strict(finNode, current_x, len); % New strict helper
                                    if ~isempty(finData)
                                        fin_sets_data{end+1} = finData;
                                        fprintf('      Extracted FinSet: %s (Num=%d, RootC=%.3f, TipC=%.3f, Span=%.3f, SweepL=%.3f, TE Pos=%.3f)\n', ...
                                            finData.name, finData.num, finData.root_chord, finData.tip_chord, finData.span_length, finData.sweep_length, finData.position_TE);
                                    end
                                end
                            end
                        end

                    case 'transition'
                        len = str2double(getNodeValue_Strict(node, 'length', '0'));
                        foreRadiusStr = getNodeValue_Strict(node, 'foreradius', 'auto');
                        aftRadiusStr = getNodeValue_Strict(node, 'aftradius', 'auto');
                        shapeType = getNodeValue_Strict(node, 'shape', 'conical');
                        foreRadius = parseRadius_Strict(foreRadiusStr, current_radius);
                        aftRadius = parseRadius_Strict(aftRadiusStr, foreRadius * 0.8); % Default from ork3d

                        fprintf('    Transition Params: L=%.3f, R_fore=%.3f, R_aft=%.3f, Type=%s\n', len, foreRadius, aftRadius, shapeType);
                        if abs(foreRadius - current_radius) > 1e-6 && current_radius ~= 0
                             fprintf('      Radius mismatch warning (%.4f -> %.4f).\n', current_radius, foreRadius);
                        end
                        start_radius_actual = current_radius;

                        component_len = len;
                        component_end_radius = aftRadius;

                    otherwise
                        fprintf('    Unsupported component type: %s. Calculating length only.\n', nodeName);
                        lenStr = getNodeValue_Strict(node, 'length', '0');
                        component_len = str2double(lenStr);
                        if isnan(component_len), component_len = 0; end
                        component_end_radius = current_radius;
                        fprintf('      Length (estimated): %.3f\n', component_len);
                end

                % --- 3.3. 상태 업데이트 ---
                absolute_end_x = absolute_start_x + component_len;
                all_component_end_x(end+1) = absolute_end_x;
                current_absolute_end_x = absolute_end_x;
                current_radius = component_end_radius;
                if current_radius > max_radius, max_radius = current_radius; end
                fprintf('    Processing complete. Absolute End X: %.4f m, End Radius: %.4f m\n', absolute_end_x, current_radius);

            end % if Element Node
        end % for childNodes loop

        % --- 4. 최종 파라미터 정리 ---
        if ~isempty(all_component_end_x)
            rocketParams.vehicle.L = max(all_component_end_x);
        else, rocketParams.vehicle.L = 0; end
        rocketParams.vehicle.D_ref = max_radius * 2;
        rocketParams.vehicle.r_ref = max_radius;

        if ~isempty(fin_sets_data)
            rocketParams.fin = fin_sets_data{1}; % Select first fin set
            % Calculate sweep angle (Leading Edge)
            if isfield(rocketParams.fin, 'span_length') && rocketParams.fin.span_length > 1e-6
                rocketParams.fin.sweep_angle = atand(rocketParams.fin.sweep_length / rocketParams.fin.span_length);
            else, rocketParams.fin.sweep_angle = 0; end
            fprintf('\nSelected main fin set for parameters: %s\n', rocketParams.fin.name);
        else
            rocketParams.fin = struct();
            fprintf('\nWarning: No fin sets found for parameter output.\n');
        end

        fprintf('\n--- Final Extracted Parameters --- \n');
        fprintf('Total Length (L): %.4f m\n', rocketParams.vehicle.L);
        fprintf('Max Diameter (D_ref): %.4f m\n', rocketParams.vehicle.D_ref);
        if isfield(rocketParams.fin, 'num')
            fprintf('Fin Count: %d\n', rocketParams.fin.num);
            fprintf('Fin Root Chord: %.4f m\n', rocketParams.fin.root_chord);
            fprintf('Fin Tip Chord: %.4f m\n', rocketParams.fin.tip_chord);
            fprintf('Fin Span/Height: %.4f m\n', rocketParams.fin.span_length);
            fprintf('Fin Sweep Length: %.4f m\n', rocketParams.fin.sweep_length);
            fprintf('Fin Sweep Angle (LE): %.2f deg\n', rocketParams.fin.sweep_angle);
            fprintf('Fin TE Position: %.4f m\n', rocketParams.fin.position_TE);
        end
         if isfield(rocketParams, 'nosecone')
            fprintf('Nosecone Length: %.3f m\n', rocketParams.nosecone.length);
            fprintf('Nosecone Type: %s\n', rocketParams.nosecone.type);
         end

    catch ME
        fprintf(2, '\n*** Error during strict parameter extraction ***\n');
        fprintf(2, 'Error Message: %s\n', ME.message);
        fprintf(2, 'Location: %s, Line: %d\n', ME.stack(1).name, ME.stack(1).line);
        rocketParams = struct(); % Return empty on error
    end
end

% =========================================================================
% Helper Functions (Strictly based on ork2matlab3d, plotting removed)
% =========================================================================

function [rocketName, designer] = getRocketInfo_Strict(xmldata)
    % Based on ork2matlab3d getRocketInfo
    rocketName = 'Unknown Rocket'; designer = 'Unknown';
    rocketNodeList = getChildElementsByTagName_Strict(xmldata, 'rocket'); % Use strict helper
    if isempty(rocketNodeList), return; end % Return default if no rocket tag
    rocketNode = rocketNodeList.item(0);
    if isempty(rocketNode), return; end
    rocketName = getNodeValue_Strict(rocketNode, 'name', 'Name Not Found'); % Use strict helper
    designer = getNodeValue_Strict(rocketNode, 'designer', 'Unknown'); % Use strict helper
end

function nodeValue = getNodeValue_Strict(parentNode, tagName, defaultValue)
    % Based on ork2matlab3d getNodeValue
    nodeValue = defaultValue;
    if isempty(parentNode), return; end % Handle empty parent
    nodeList = getChildElementsByTagName_Strict(parentNode, tagName); % Use strict helper
    % Check getLength before item(0)
    listLength = 0; if ~isempty(nodeList), try listLength = nodeList.getLength(); catch, listLength=0; end, end
    if listLength > 0
        firstItem = nodeList.item(0);
        % Check item and first child before getting text content
        if ~isempty(firstItem) && ismethod(firstItem, 'getFirstChild') && ~isempty(firstItem.getFirstChild()) ...
           && ismethod(firstItem.getFirstChild, 'getNodeValue')
            nodeValue = strtrim(char(firstItem.getFirstChild().getNodeValue())); % Use getNodeValue and trim
        end
    end
end

function subCompNode = findMainComponentListNode_Strict(xmldata)
    % Based on ork2matlab3d findMainComponentListNode
    subCompNode = [];
    rocketNodeList = getChildElementsByTagName_Strict(xmldata, 'rocket');
    if isempty(rocketNodeList), fprintf('Error: <rocket> not found.\n'); return; end
    rocketNode = rocketNodeList.item(0); if isempty(rocketNode), return; end

    rocketSubNode = findDirectChildElement_Strict(rocketNode, 'subcomponents'); % Use strict helper
    if isempty(rocketSubNode), fprintf('Error: <rocket>/<subcomponents> not found.\n'); return; end

    mainStageNode = findDirectChildElement_Strict(rocketSubNode, 'stage'); % Use strict helper
    if isempty(mainStageNode), fprintf('Error: <rocket>/<subcomponents>/<stage> not found.\n'); return; end

    subCompNode = findDirectChildElement_Strict(mainStageNode, 'subcomponents'); % Use strict helper
    if isempty(subCompNode), fprintf('Error: <stage>/<subcomponents> (component list) not found.\n'); end
end

function childElement = findDirectChildElement_Strict(parentNode, tagName)
    % Based on ork2matlab3d findDirectChildElement
    childElement = [];
    if isempty(parentNode) || ~ismethod(parentNode, 'getChildNodes'), return; end
    childNodes = parentNode.getChildNodes;
    for k = 0:childNodes.getLength-1
        child = childNodes.item(k);
        nodeType = -1; if ~isempty(child) && ismethod(child, 'getNodeType'), nodeType = child.getNodeType(); end
        % Check type and name
        if nodeType == org.w3c.dom.Node.ELEMENT_NODE && strcmp(char(child.getNodeName), tagName)
            childElement = child;
            return; % Return first match
        end
    end
end

function childElements = getChildElementsByTagName_Strict(parentNode, tagName)
    % Based on ork2matlab3d getChildElementsByTagName, returns NodeList or []
    childElements = []; % Default to MATLAB empty array
    if isempty(parentNode) || ~ismethod(parentNode, 'getElementsByTagName'), return; end
    try
        tempList = parentNode.getElementsByTagName(tagName);
        % Basic check if it returned something that has getLength method
        if ~isempty(tempList) && ismethod(tempList, 'getLength')
            childElements = tempList;
        end
    catch ME
        % fprintf('Warning: Error in getElementsByTagName_Strict for "%s": %s\n', tagName, ME.message);
    end
end

function [start_x, position_type_desc] = getAbsoluteStartPosition_Strict(componentNode, default_start_x)
    % Based on ork2matlab3d getAbsoluteStartPosition
    start_x = default_start_x; position_type_desc = 'Sequential';
    posList = getChildElementsByTagName_Strict(componentNode, 'position');
    listLength = 0; if ~isempty(posList), try listLength = posList.getLength(); catch, listLength=0; end, end

    if listLength > 0
        posNode = posList.item(0);
        if ~isempty(posNode)
             posValueStr = '';
             if ismethod(posNode, 'getFirstChild') && ~isempty(posNode.getFirstChild())
                 try posValueStr = char(posNode.getFirstChild().getNodeValue()); catch, posValueStr=''; end
             end
             posValue = str2double(posValueStr); if isnan(posValue), posValue=0; end

             posType = '';
             if ismethod(posNode, 'getAttribute')
                 try posType = char(posNode.getAttribute('type')); catch, posType=''; end
             end

            if strcmpi(posType, 'absolute')
                start_x = posValue; position_type_desc = sprintf('Absolute (%.4f)', posValue);
            else
                % Original code assumed sequential for relative, let's stick to that for strictness
                position_type_desc = sprintf('Relative type "%s" - Sequential Applied', posType);
                % start_x = default_start_x + posValue; % If relative offset needed
            end
        end
    end
end

function radius = parseRadius_Strict(radiusStr, default_radius)
    % Based on ork2matlab3d parseRadius
    radius = default_radius; % Start with default
    if isempty(radiusStr) || ~ischar(radiusStr), return; end
    radiusStr = strtrim(radiusStr);
    if isempty(radiusStr), return; end

    try
        if startsWith(radiusStr, 'auto', 'IgnoreCase', true)
            parts = strsplit(radiusStr);
            if length(parts) >= 2
                val = str2double(parts{2});
                if ~isnan(val), radius = val; end % Use value if valid number
            end
        else
            val = str2double(radiusStr);
            if ~isnan(val), radius = val; end % Use value if valid number
        end
    catch
        % Ignore parsing errors, keep default
    end

    % Final check from original code (adjust upper limit if needed)
    if radius < 0 || radius > 10 || isnan(radius)
        % fprintf('    Warning: Invalid radius value (%.4f) from "%s". Using default (%.4f).\n', radius, radiusStr, default_radius);
        radius = default_radius;
    end
end

function finData = extractFinData_Strict(finSetNode, parent_start_x, parent_len)
    % Extracts fin parameters based on processFinSet3D logic
    finData = []; % Return empty if fails
    try
        finData.name = getNodeValue_Strict(finSetNode, 'name', 'Unnamed Fins');
        finData.root_chord = str2double(getNodeValue_Strict(finSetNode, 'rootchord', '0'));
        finData.tip_chord = str2double(getNodeValue_Strict(finSetNode, 'tipchord', '0'));
        finData.span_length = str2double(getNodeValue_Strict(finSetNode, 'height', '0')); % height is span
        finData.sweep_length = str2double(getNodeValue_Strict(finSetNode, 'sweeplength', '0'));
        finData.thickness = str2double(getNodeValue_Strict(finSetNode, 'thickness', '0.001'));
        % Attempt to get fin count directly from the finset node
        finData.num = str2double(getNodeValue_Strict(finSetNode, 'fincount', '4')); % Default 4
        if isnan(finData.num) || finData.num <= 0, finData.num = 4; end

        if finData.root_chord <= 0 || finData.span_length < 0
            fprintf('  Warning: Invalid fin dimensions for "%s". Skipping.\n', finData.name);
            finData = []; return;
        end

        % Position calculation (TE Root X) based on processFinSet3D logic
        ref_coord_x = parent_start_x; % Default
        posList = getChildElementsByTagName_Strict(finSetNode, 'position');
        listLength = 0; if ~isempty(posList), try listLength = posList.getLength(); catch, listLength=0; end, end

        if listLength > 0
            posNode = posList.item(0);
            if ~isempty(posNode)
                 posValueStr = ''; if posNode.hasChildNodes && ~isempty(posNode.getFirstChild()), try posValueStr = char(posNode.getFirstChild().getNodeValue()); catch, posValueStr=''; end, end
                 finPosVal = str2double(posValueStr); if isnan(finPosVal), finPosVal=0; end

                 finPosType = ''; if ismethod(posNode, 'getAttribute'), try finPosType = char(posNode.getAttribute('type')); catch, finPosType=''; end, end

                 parent_abs_end_x = parent_start_x + parent_len;
                 parent_abs_mid_x = parent_start_x + parent_len / 2;
                 switch lower(finPosType)
                     case 'bottom', ref_coord_x = parent_abs_end_x + finPosVal;
                     case 'top', ref_coord_x = parent_start_x + finPosVal;
                     case 'middle', ref_coord_x = parent_abs_mid_x + finPosVal;
                     case 'absolute', ref_coord_x = finPosVal;
                     otherwise, ref_coord_x = parent_start_x + finPosVal; % Default to top relative
                 end
            end
        % else: ref_coord_x remains parent_start_x (default from ork3d)
        end
        finData.position_TE = ref_coord_x;

    catch ME
        fprintf('  Error extracting strict fin data for "%s": %s\n', getNodeValue_Strict(finSetNode, 'name', 'Unnamed'), ME.message);
        finData = [];
    end
end
