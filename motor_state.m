function compList_motor = motor_state(flatRocketParams, currentTime, csvFilePath)
% motor_state  현재 시간에 따른 모터의 질량 특성 계산 (단순화된 버전)
%
% compList_motor = motor_state(flatRocketParams, currentTime, csvFilePath)
%
% 입력
% flatRocketParams : flattenRocketStructure로 얻은 평탄화된 로켓 파라미터 구조체
% currentTime : 현재 시뮬레이션 시간 [s]
% csvFilePath : 모터 데이터 CSV 파일 경로 (예: 'AeroTech_M2400T.csv')
%
% 출력
% compList_motor : 모터의 질량 특성 정보를 담은 구조체

% persistent 변수 선언 - 함수 호출 간 값 유지
persistent motorData motorIdx motorGeom initialized
persistent timeArray propellantMassArray totalMass structuralMass initialPropellantMass
persistent motorName motorPath

% 처음 호출되거나 CSV 파일이 변경된 경우 초기화
if isempty(initialized) || ~strcmp(motorPath, csvFilePath)
    motorPath = csvFilePath; % 현재 CSV 파일 경로 저장
    
    % 하드코딩된 모터 정보
    motorName = 'AeroTech M2400T';
    totalMass = 6.451; % [kg] - 모터 초기 총 질량(추진제+케이스)
    structuralMass = 1.449; % [kg] - 구조체 질량 (하드코딩)
    initialPropellantMass = totalMass - structuralMass; % [kg] - 초기 추진제 질량
    
    % 모터 컴포넌트 찾기 (한 번만 수행)
    motorIdx = [];
    for k = 1:numel(flatRocketParams)
        if strcmpi(getSafe(flatRocketParams(k), 'Type', ''), 'motor')
            motorIdx = k;
            break;
        end
    end
    
    if isempty(motorIdx)
        fprintf('경고: 모터 컴포넌트를 찾을 수 없습니다. 빈 모터 구조체를 반환합니다.\n');
        compList_motor = struct('name', motorName, 'type', 'motor', ...
                             'mass', 0, 'cg_ref', [0, 0, 0], 'inertia_cg', zeros(3));
        return;
    end
    
    % 기본 모터 기하학적 정보 (간소화)
    c = flatRocketParams(motorIdx);
    motorGeom = struct();
    motorGeom.L = getSafe(c.Geometry, 'Length', 0.597);
    motorGeom.X = getSafe(c.Position, 'AbsoluteStartX', 0);
    
    % CSV 파일 읽기 (한 번만 수행)
    try
        opts = detectImportOptions(csvFilePath, 'VariableNamingRule', 'preserve');
        motorData = readtable(csvFilePath, opts);
        
        % 시간 배열 추출
        timeArray = motorData.("Time(s)");
        
        % CSV는 추진제 질량만 포함 (g -> kg 변환)
        propellantMassArray = motorData.("Mass(g)")/1000;
        
        % 초기 추진제 질량 비율 계산 및 스케일링
        scaleFactor = initialPropellantMass / propellantMassArray(1);
        propellantMassArray = propellantMassArray * scaleFactor;
        
        % 안전 검사: 최종 추진제 질량이 0에 가까운지 확인
        if propellantMassArray(end) > 0.1 * initialPropellantMass
            fprintf('경고: CSV 데이터의 최종 추진제 질량이 초기 질량의 10%% 이상입니다.\n');
            fprintf('CSV가 추진제 소모 과정을 완전히 포함하지 않을 수 있습니다.\n');
        end
    catch e
        fprintf('경고: CSV 파일 "%s" 읽기 실패: %s\n', csvFilePath, e.message);
        fprintf('기본 추진제 감소 프로필을 사용합니다.\n');
        
        % 기본값 설정 - 직선적 감소 가정
        timeArray = [0; 10]; % 임의의 시간 배열
        propellantMassArray = [initialPropellantMass; 0]; % 추진제 완전 소모
    end
    
    % 초기화 완료 표시
    initialized = true;
    fprintf('모터 데이터 초기화 완료: %s (총질량: %.3f kg, 구조체: %.3f kg, 추진제: %.3f kg)\n', ...
        motorName, totalMass, structuralMass, initialPropellantMass);
end

% 현재 시간에 따른 추진제 질량 계산 (보간)
if currentTime <= min(timeArray)
    currentPropellantMass = propellantMassArray(1);
    % 수정: t=0 일때도 동일한 CG 계산 로직 사용 - 초기 burnRatio는 항상 1.0
    burnRatio = 1.0; 
    currentCGOffset = 0.3 + 0.4 * burnRatio; % 0.3~0.7 범위 (뒤쪽->앞쪽)
elseif currentTime >= max(timeArray)
    currentPropellantMass = propellantMassArray(end);
    burnRatio = currentPropellantMass / initialPropellantMass;
    if burnRatio > 0
        currentCGOffset = 0.3 + 0.4 * burnRatio;
    else
        currentCGOffset = 0.3 + 0.4 * burnRatio; % 추진제 없음
    end
else
    currentPropellantMass = interp1(timeArray, propellantMassArray, currentTime, 'linear');
    
    % 간단한 CG 추정 (추진제가 앞쪽부터 소모된다고 가정)
    burnRatio = currentPropellantMass / initialPropellantMass;
    if burnRatio > 0
        currentCGOffset = 0.3 + 0.4 * burnRatio; % 0.3~0.7 범위 (뒤쪽->앞쪽)
    end
end

% 안전을 위한 음수 검사
currentPropellantMass = max(0, currentPropellantMass);

% 현재 모터 총 질량 계산 (추진제 + 구조체)
currentMass = structuralMass + currentPropellantMass;

% 디버깅용 출력
% fprintf('시간 %.2f초: 모터 총질량 = %.3f kg (구조체: %.3f kg, 추진제: %.3f kg)\n', ...
%        currentTime, currentMass, structuralMass, currentPropellantMass);

% CG 위치 계산 (단순화)
cg = [motorGeom.X + motorGeom.L * currentCGOffset, 0, 0];

% 관성 텐서 계산 (단순화 - 원통형 모델)
% 모터 전체를 하나의 균일한 원통으로 가정
D = getSafe(flatRocketParams(motorIdx).Geometry, 'Diameter', 0.098); % 직경
R = D/2; % 반지름

% 균일한 원통에 대한 간단한 관성 계산
Ixx = currentMass * R^2 / 2;
Iyy = currentMass * (3*R^2 + motorGeom.L^2) / 12;
I = diag([Ixx, Iyy, Iyy]);

% 부품 정보 구조체 생성
compList_motor = struct('name', motorName, ...
                       'type', 'motor', ...
                       'mass', currentMass, ...
                       'cg_ref', cg, ...
                       'inertia_cg', I);
end

function v = getSafe(S, field, def)
% getSafe  구조체에서 안전하게 필드 값을 추출하는 함수
v = def;
if isempty(S) || ~isstruct(S), return, end

if iscell(field)
    for f = field
        if ischar(f{1}) || isstring(f{1})
            tmp = getSafe(S, f{1}, NaN);
            if ~isnan(tmp), v = tmp; return, end
        end
    end
    return
end

if ~(ischar(field) || isstring(field)), return, end

parts = strsplit(char(field), '.'); 
tmp = S;
for p = parts
    if isstruct(tmp) && isfield(tmp, p{1})
        tmp = tmp.(p{1});
    else
        return
    end
end
v = tmp;
end