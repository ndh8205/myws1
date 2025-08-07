function params = vehicle_params(flatRocketParams, t, motorCsvPath, reset)
% vehicle_params  로켓 파라미터 관리 함수 - 좌표계 방향 수정
%
% params = vehicle_params(flatRocketParams, t, motorCsvPath, reset)
%
% 입력:
% flatRocketParams : 평탄화된 로켓 파라미터 구조체
% t : 현재 시뮬레이션 시간 [s]
% motorCsvPath : 모터 CSV 파일 경로
% reset : (선택) 초기화 플래그 (기본값: false)
%
% 출력:
% params : 업데이트된 로켓 파라미터 구조체
%
% 좌표계 설명:
% - 설계 좌표계: 노즈팁이 원점, 노즈→노즐 방향이 +X
% - 바디 좌표계: CG가 원점, CG→노즈 방향이 +X (설계 좌표계와 반대 방향)

% persistent 변수 선언 - 함수 호출 간 값 유지
persistent flat_params static_cache initial_mass update_count is_initialized
persistent initial_cg timeArray thrustArray
persistent rocket_diameter rocket_length rocket_surface_area
persistent nose_tip_abs nozzle_abs canard_abs rcs_abs

% 입력 검증 및 기본값 설정
if nargin < 3
    error('vehicle_params:입력부족', '최소 3개 입력이 필요합니다: flatRocketParams, t, motorCsvPath');
end

if nargin < 4
    reset = false;
end

% 기본 파라미터 구조체 생성
params = struct();

%% 환경 파라미터 (고정값)
params.environment.g = [0, 0, 9.81]'; % [m/s^2]
params.environment.ro = 1.666; % [kg/m^3]
params.environment.ro_2 = 1.24; % [kg/m^3]
% Spaceport America 환경 파라미터
params.environment.latitude = 32.93952; % [degrees]
params.environment.longitude = -106.92006; % [degrees]
params.environment.altitude = 1401; % [m] Elevation
% 평균 기상 조건
params.environment.wind_speed = 6.0; % [m/s]
params.environment.pressure = 1025; % [hPa]
params.environment.temperature = 29; % [°C]
params.environment.temp_range = [21, 35]; % [°C]

%% 제어기 파라미터 (고정값)
params.control.Ka_pp = [0, 0, 0]'; % Attitude P
params.control.Ka_p = [0, 0, 0]'; % Rate P
params.control.Ka_i = [0, 0, 0]'; % Rate I
params.control.Ka_d = [0, 0, 0]'; % Rate D

%% 초기화 로직 (첫 호출 또는 reset=true인 경우)
if reset || isempty(is_initialized) || isempty(static_cache)
    fprintf('vehicle_params: 초기화 수행 중...\n');
    
    % 평탄화된 구조체 저장
    flat_params = flatRocketParams;
    
    % 추력 데이터 로드 (CSV 파일에서)
    try
        opts = detectImportOptions(motorCsvPath, 'VariableNamingRule', 'preserve');
        motorData = readtable(motorCsvPath, opts);
        timeArray = motorData.("Time(s)");
        thrustArray = motorData.("Thrust(N)");
        fprintf('모터 CSV 로드 완료: 시간 범위 [%.2f, %.2f]초\n', ...
                min(timeArray), max(timeArray));
    catch e
        warning('모터 CSV 파일 로드 실패: %s', E.message);
        % 기본값 설정
        timeArray = [0; 10];
        thrustArray = [2000; 0];
    end
    
    % 로켓 설계 좌표계의 주요 지점 위치 추출
    [rocket_diameter, rocket_length, rocket_surface_area, nose_tip_x, nozzle_x, canard_x, rcs_x] = extractRocketGeometry(flat_params);
    
    % 설계 좌표계의 절대 위치 저장
    nose_tip_abs = [nose_tip_x, 0, 0];
    nozzle_abs = [nozzle_x, 0, 0];
    canard_abs = [canard_x, 0, 0];
    rcs_abs = [rcs_x, 0, 0];
    
    % vehicle_mass_calc 첫 호출 - 정적 부품 캐시 및 초기 질량 계산
    [initial_mass, initial_cg_calc, ~, components] = vehicle_mass_calc(flat_params, 0, motorCsvPath);
    initial_cg = initial_cg_calc; % 초기 CG 저장
    static_cache = components{1};  % 정적 부품 캐시 저장
    
    fprintf('설계 좌표계 주요 지점 위치:\n');
    fprintf('  노즈팁 (원점): [%.3f, 0, 0] m\n', nose_tip_x);
    fprintf('  로켓 노즐: [%.3f, 0, 0] m\n', nozzle_x);
    fprintf('  Canard 시스템: [%.3f, 0, 0] m\n', canard_x);
    fprintf('  RCS 시스템: [%.3f, 0, 0] m\n', rcs_x);
    fprintf('  초기 CG: [%.3f, %.3f, %.3f] m\n', initial_cg);
    
    fprintf('로켓 기하학적 파라미터:\n');
    fprintf('  직경: %.3f m\n', rocket_diameter);
    fprintf('  길이: %.3f m\n', rocket_length);
    fprintf('  표면적: %.4f m²\n', rocket_surface_area);
    
    % 업데이트 카운터 초기화
    update_count = 0;
    
    % 초기화 완료 표시
    is_initialized = true;
    fprintf('vehicle_params 초기화 완료\n');
end

%% 현재 시간에서의 질량 특성 계산 (vehicle_mass_calc 활용)
[current_mass, current_cg, inertia_tensor] = vehicle_mass_calc(flat_params, t, motorCsvPath, static_cache);

% CG 변화량 계산
cg_delta = current_cg - initial_cg;

%% 현재 추력 값 계산 (보간)
if t <= min(timeArray)
    current_thrust = thrustArray(1);
elseif t >= max(timeArray)
    current_thrust = thrustArray(end);
else
    current_thrust = interp1(timeArray, thrustArray, t, 'linear');
    if isnan(current_thrust)
        current_thrust = 0;
    end
end

%% Vehicle Parameters - 질량 특성 매핑
% 기본 질량 특성
params.vehicle.m_W = current_mass;      % 현재 총 질량
params.vehicle.m_W_const = initial_mass; % 초기 총 질량
params.vehicle.thrust = current_thrust;  % 현재 추력

% CG 좌표 정보 (두 좌표계 모두 제공)
params.vehicle.CG_abs = current_cg;   % 설계 좌표계에서 CG 위치
params.vehicle.CG = [0, 0, 0];        % 바디 좌표계에서는 항상 [0,0,0]
params.vehicle.CG_delta = cg_delta;   % 초기 CG에서의 변화량

% CG 기준 좌표계로 표현 (항상 [0,0,0])
params.vehicle.CG_x = 0;  % 바디 좌표계이므로 0
params.vehicle.CG_y = 0;  % 바디 좌표계이므로 0
params.vehicle.CG_z = 0;  % 바디 좌표계이므로 0

% 관성 모멘트 성분
params.vehicle.J_X = inertia_tensor(1,1);
params.vehicle.J_Y = inertia_tensor(2,2);
params.vehicle.J_Z = inertia_tensor(3,3);
params.vehicle.J = inertia_tensor;  % 3x3 텐서

% 로켓 기하학적 파라미터 (추출된 값 사용)
params.vehicle.D_ref = rocket_diameter;          % [m] % 로켓 직경
params.vehicle.r_ref = rocket_diameter/2;        % [m] % 로켓 반경
params.vehicle.L = rocket_length;                % [m] % 로켓 길이
params.vehicle.S_W_ref = rocket_surface_area;    % [m^2] % 표면적
params.vehicle.S_A_ref = pi * (rocket_diameter/2)^2;  % [m^2] % 단면적

% 바디 좌표계로 변환 (CG 기준, CG→노즈 방향이 +X)
% 설계 좌표계: 노즈→노즐 방향이 +X
% 바디 좌표계: CG→노즈 방향이 +X (반대 방향)
% 따라서, 설계 좌표계에서의 상대 위치를 계산한 후 X축 부호를 반전

% CG에서 노즐까지의 벡터 (설계 좌표계에서 계산 후 부호 반전)
params.vehicle.Lt = -(nozzle_abs(1) - current_cg(1));

% CG에서 카나드까지의 벡터 (설계 좌표계에서 계산 후 부호 반전)
params.vehicle.Lc = -(canard_abs(1) - current_cg(1));

% CG에서 RCS 시스템까지의 벡터 (설계 좌표계에서 계산 후 부호 반전)
params.vehicle.Lrcs = -(rcs_abs(1) - current_cg(1));

% RCS 추력값 (기본값)
params.vehicle.RCS_T = 3;  % [N]

% 시간 정보
params.time = t;

if update_count == 1400
% 디버그: 바디 좌표계로 변환된 값 확인
fprintf('시간 %.3f초, CG=[%.3f, 0, 0]m에서의 상대 위치(바디 좌표계):\n', t, current_cg(1));
fprintf('  Lt (CG→노즐): %.3f m %s\n', params.vehicle.Lt, iff(params.vehicle.Lt < 0, '(뒤쪽)', '(앞쪽)'));
fprintf('  Lc (CG→카나드): %.3f m %s\n', params.vehicle.Lc, iff(params.vehicle.Lc > 0, '(앞쪽)', '(뒤쪽)'));
fprintf('  Lrcs (CG→RCS): %.3f m %s\n', params.vehicle.Lrcs, iff(params.vehicle.Lrcs > 0, '(앞쪽)', '(뒤쪽)'));
end
% 업데이트 카운트 증가
update_count = update_count + 1;

% 디버그 정보
params.debug.static_cache_size = numel(static_cache);
params.debug.update_count = update_count;
params.debug.initial_cg = initial_cg;
params.debug.current_cg = current_cg;
params.debug.cg_delta = cg_delta;
params.debug.design_frame = struct('nose_tip', nose_tip_abs, 'nozzle', nozzle_abs, ...
                                  'canard', canard_abs, 'rcs', rcs_abs);
params.debug.body_frame = struct('Lt', params.vehicle.Lt, 'Lc', params.vehicle.Lc, ...
                                'Lrcs', params.vehicle.Lrcs);
end

% 삼항 연산자 유틸리티 함수
function result = iff(condition, true_value, false_value)
    if condition
        result = true_value;
    else
        result = false_value;
    end
end


