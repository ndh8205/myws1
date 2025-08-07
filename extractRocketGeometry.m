function [diameter, length, surface_area, nose_tip_x, nozzle_x, canard_x, rcs_x] = extractRocketGeometry(flatRocketParams)
% extractRocketGeometry  로켓의 기하학적 파라미터 추출 함수
%
% [diameter, length, surface_area, nose_tip_x, nozzle_x, canard_x, rcs_x] = extractRocketGeometry(flatRocketParams)
%
% 입력:
% flatRocketParams : 평탄화된 로켓 파라미터 구조체
%
% 출력:
% diameter : 로켓 최대 직경 [m]
% length : 로켓 전체 길이 [m]
% surface_area : 로켓 표면적 [m^2]
% nose_tip_x : 노즈팁 X 좌표 (설계 좌표계 원점) [m]
% nozzle_x : 엔진 노즐 X 좌표 (설계 좌표계) [m]
% canard_x : Canard 시스템 X 좌표 (설계 좌표계) [m]
% rcs_x : RCS 시스템 X 좌표 (설계 좌표계) [m]

    % 초기값 설정
    diameter = 0;
    length = 0;
    surface_area = 0;
    min_x = Inf;  % 로켓 앞쪽 끝(노즈팁)
    max_x = -Inf; % 로켓 뒤쪽 끝(노즐)
    nozzle_x = NaN;
    canard_x = NaN;
    rcs_x = NaN;
    
    % 최대 반경(직경) 찾기
    max_radius = 0;
    
    % 표면적 계산 변수
    total_surface_area = 0;
    
    % 테일콘 정보 저장 변수
    tailcone_start = NaN;
    tailcone_length = NaN;
    
    % 각 부품 반복 처리
    for i = 1:numel(flatRocketParams)
        comp = flatRocketParams(i);
        
        % 위치 정보 추출
        start_x = NaN;
        end_x = NaN;
        
        if isfield(comp, 'Position') && isfield(comp.Position, 'AbsoluteStartX')
            start_x = comp.Position.AbsoluteStartX;
        end
        
        if isfield(comp, 'Position') && isfield(comp.Position, 'AbsoluteEndX')
            end_x = comp.Position.AbsoluteEndX;
        elseif isfield(comp, 'Position') && isfield(comp.Position, 'AbsoluteStartX') && isfield(comp.Position, 'Length')
            if isfield(comp.Position, 'Length')
                end_x = comp.Position.AbsoluteStartX + comp.Position.Length;
            end
        end
        
        % 최대/최소 X 위치 업데이트
        if ~isnan(start_x) && start_x < min_x
            min_x = start_x;
        end
        
        if ~isnan(end_x) && end_x > max_x
            max_x = end_x;
        end
        
        % 반경 정보 추출
        radius = 0;
        
        % 부품 타입에 따라 반경 추출 및 처리
        if strcmpi(getSafe(comp, 'Type', ''), 'bodytube')
            if isfield(comp.Geometry, 'Radius')
                radius = comp.Geometry.Radius;
            elseif isfield(comp.Geometry, 'OuterRadius')
                radius = comp.Geometry.OuterRadius;
            elseif isfield(comp.Geometry, 'Diameter')
                radius = comp.Geometry.Diameter / 2;
            end
            
            % 표면적 추가 (2*π*r*l)
            if ~isnan(radius) && ~isnan(start_x) && ~isnan(end_x)
                comp_length = end_x - start_x;
                comp_surface = 2 * pi * radius * comp_length;
                total_surface_area = total_surface_area + comp_surface;
            end
        elseif strcmpi(getSafe(comp, 'Type', ''), 'nosecone')
            if isfield(comp.Geometry, 'AftRadius')
                radius = comp.Geometry.AftRadius;
            elseif isfield(comp.Geometry, 'BaseRadius')
                radius = comp.Geometry.BaseRadius;
            elseif isfield(comp.Geometry, 'BaseDiameter')
                radius = comp.Geometry.BaseDiameter / 2;
            end
            
            % 표면적 추가 (근사값: π*r*l)
            if ~isnan(radius) && ~isnan(start_x) && ~isnan(end_x)
                comp_length = end_x - start_x;
                comp_surface = pi * radius * comp_length;  % 간단한 근사
                total_surface_area = total_surface_area + comp_surface;
            end
        elseif strcmpi(getSafe(comp, 'Type', ''), 'transition')
            % 테일콘 찾기
            if isfield(comp, 'Name') && strcmpi(comp.Name, 'Tailcone')
                tailcone_start = start_x;
                
                % 길이 정보 추출
                if isfield(comp.Geometry, 'Length')
                    tailcone_length = comp.Geometry.Length;
                elseif ~isnan(start_x) && ~isnan(end_x)
                    tailcone_length = end_x - start_x;
                end
            end
            
            % 전방 반경과 후방 반경 중 큰 값 사용
            fore_radius = 0;
            aft_radius = 0;
            
            if isfield(comp.Geometry, 'ForeRadius')
                fore_radius = comp.Geometry.ForeRadius;
            end
            
            if isfield(comp.Geometry, 'AftRadius')
                aft_radius = comp.Geometry.AftRadius;
            end
            
            radius = max(fore_radius, aft_radius);
            
            % 표면적 추가 (원뿔대 근사값)
            if ~isnan(fore_radius) && ~isnan(aft_radius) && ~isnan(start_x) && ~isnan(end_x)
                comp_length = end_x - start_x;
                slant_height = sqrt((fore_radius-aft_radius)^2 + comp_length^2);
                comp_surface = pi * (fore_radius + aft_radius) * slant_height;
                total_surface_area = total_surface_area + comp_surface;
            end
        elseif strcmpi(getSafe(comp, 'Type', ''), 'masscomponent')
            % 특정 이름을 가진 MassComponent 찾기
            if isfield(comp, 'Name')
                % Canard_System 질량 구성 요소
                if contains(comp.Name, 'Canard_System')
                    canard_x = start_x; % 시작 위치 사용
                end
                
                % RCS Thruster 질량 구성 요소
                if contains(comp.Name, 'RCS Thruster_Weight')
                    rcs_x = start_x; % 시작 위치 사용
                end
            end
        end
        
        % 최대 반경 업데이트
        if radius > max_radius
            max_radius = radius;
        end
    end
    
    % 테일콘의 끝을 노즐 위치로 설정
    if ~isnan(tailcone_start) && ~isnan(tailcone_length)
        nozzle_x = tailcone_start + tailcone_length;
    else
        nozzle_x = max_x; % 기본값: 로켓의 끝
    end
    
    % 최종 결과 계산
    diameter = max_radius * 2;
    length = max_x - min_x;
    surface_area = total_surface_area;
    
    % 설계 좌표계 원점 설정 (노즈팁)
    nose_tip_x = min_x;
    
    % 결과 검증 및 기본값 설정
    if isinf(min_x) || isinf(max_x)
        min_x = 0;
        max_x = 2.679; % 로그에서 확인된 로켓 길이
        length = 2.679;
        nose_tip_x = 0;
        fprintf('경고: 로켓 범위를 판단할 수 없습니다. 기본값 [0, 2.679]m 사용\n');
    end
    
    if diameter <= 0
        diameter = 0.136; % 로그에서 확인된 로켓 직경
        fprintf('경고: 로켓 직경을 판단할 수 없습니다. 기본값 136mm 사용\n');
    end
    
    if isnan(nozzle_x)
        nozzle_x = 2.679; % 로그에서 확인된 로켓 길이
        fprintf('경고: 노즐 위치를 찾을 수 없습니다. 로켓 끝 위치 사용\n');
    end
    
    if isnan(canard_x)
        fprintf('경고: "Canard_System" 구성 요소를 찾을 수 없습니다. 기본값 사용\n');
        canard_x = 0.692; % 로그에서 확인된 Canard 위치
    end
    
    if isnan(rcs_x)
        fprintf('경고: "RCS Thruster_Weight" 구성 요소를 찾을 수 없습니다. 기본값 사용\n');
        rcs_x = 0.895; % 로그에서 확인된 RCS 위치
    end
    
    % 최종 계산된 위치 출력
    fprintf('추출된 구성 요소 위치 (설계 좌표계):\n');
    fprintf('  노즈팁: %.3f m\n', nose_tip_x);
    fprintf('  노즐 (테일콘 끝): %.3f m\n', nozzle_x);
    fprintf('  Canard 시스템: %.3f m\n', canard_x);
    fprintf('  RCS 시스템: %.3f m\n', rcs_x);
end

function v = getSafe(S, field, def)
% getSafe  구조체에서 안전하게 필드 값을 추출하는 함수
%
% v = getSafe(S, field, def)
%
% 입력:
% S: 대상 구조체
% field: 추출할 필드 이름 ('a.b.c' 형식의 경로 또는 {'A', 'B'} 형식의 대안 목록)
% def: 필드가 없거나 값이 비어있을 경우 반환할 기본값
%
% 출력:
% v: 추출된 값 또는 기본값

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