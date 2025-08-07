function aero = aero_coefficients(params, alpha, mach, pm, qm, rm, reset)
% aero_coefficients  로켓 공력 계수 계산 함수
%
% aero = aero_coefficients(params, alpha, mach, pm, qm, rm, reset)
%
% 입력:
% params : vehicle_params 함수에서 반환된 로켓 파라미터 구조체
% alpha  : 총 받음각 [도]
% mach   : 마하수
% pm     : 롤 각속도 [rad/s]
% qm     : 피치 각속도 [rad/s]
% rm     : 요 각속도 [rad/s]
% reset  : (선택) 초기화 플래그 (기본값: false)
%
% 출력:
% aero : 12개 공력 계수가 포함된 구조체

% persistent 변수 선언 - 함수 호출 간 값 유지
persistent aero_data alpha_range mach_range cm_slopes is_initialized

% 입력 검증 및 기본값 설정
if nargin < 6
    error('aero_coefficients:입력부족', '최소 6개 입력이 필요합니다: params, alpha, mach, pm, qm, rm');
end

if nargin < 7
    reset = false;
end

% 기본 aero 구조체 생성
aero = struct();

% 초기화 로직 (첫 호출 또는 reset=true인 경우)
if reset || isempty(is_initialized) || isempty(aero_data)
    fprintf('aero_coefficients: 초기화 수행 중...\n');
    
    % 공력 데이터 로드 및 캐싱
    try
        force_moment_path = 'csv_Force_moment_axiseq.csv';  % 공력계수 CSV 파일
        opts = detectImportOptions(force_moment_path, 'VariableNamingRule', 'preserve');
        aero_data = readtable(force_moment_path, opts);

        % X축 관련 계수들의 부호 반전 - 좌표계 변환( CFD 해석진행 X축 방향 반대임 )
        % aero_coefficients 함수 수정
        % 모든 공력 계수의 일관된 변환
        aero_data.CA = -aero_data.CA;     % 축력 계수 반전
        aero_data.Clm = -aero_data.Clm;   % 롤링 모멘트 계수 반전
        aero_data.CN = -aero_data.CN;     % 법선력 계수 반전
        % aero_data.Cmm = -aero_data.Cmm;   % 피칭 모멘트 계수 반전
        aero_data.Cyawm = -aero_data.Cyawm; % 요잉 모멘트 계수 반전
        
        % 받음각과 마하수 범위 추출
        alpha_range = unique(aero_data.AoA);
        mach_range = unique(aero_data.Mach);
        
        % 기울기(미분계수) 미리 계산
        cm_slopes = calculateMomentSlopes(aero_data, alpha_range, mach_range);
        
        fprintf('공력 데이터 로드 완료: %d개 받음각, %d개 마하수\n', ...
                length(alpha_range), length(mach_range));
    catch ME
        warning('공력 데이터 로드 실패: %s\n경험적 모델로 대체합니다.', E.message);
        aero_data = [];
        alpha_range = 0:15:90;
        mach_range = [0.5, 0.8, 1.0, 1.2, 1.5, 2.0];
        cm_slopes = [];
    end
    
    % 초기화 완료 표시
    is_initialized = true;
    fprintf('aero_coefficients 초기화 완료\n');
end

% 정적 공력 계수 계산
% 공력 데이터가 있으면 보간으로 계산, 없으면 경험적 모델 사용
if ~isempty(aero_data)
    [CAm, CYm, CNm, Clm, Cmm, Cyawm] = interpolateStaticCoef(aero_data, alpha, mach, alpha_range, mach_range);
else
    % 경험적 모델 사용 (기본값)
    CAm = 0.5; 
    CYm = 0;
    CNm = 2.0 * sin(alpha * pi/180)^2;  % 간단한 경험식
    Clm = 0;
    Cmm = -0.5 * sin(2 * alpha * pi/180);  % 간단한 경험식
    Cyawm = 0;
end

% 댐핑 계수 계산
% 댐핑 미분계수 계산
[Clpm, Cmqm, Cyawrm, Clmd, Cmmd, Cyawmd] = calcDampCoef(params, alpha, mach, pm, qm, rm, aero_data, alpha_range, mach_range, cm_slopes);

% 공력 계수 구조체에 저장
% 정적 공력 계수
aero.static = struct();
aero.static.CAm = CAm;      % 축력 계수
aero.static.CYm = CYm;      % 측력 계수
aero.static.CNm = CNm;      % 법선력 계수
aero.static.Clm = Clm;      % 롤링 모멘트 계수
aero.static.Cmm = Cmm;      % 피칭 모멘트 계수
aero.static.Cyawm = Cyawm;  % 요잉 모멘트 계수

% 댐핑 미분계수
aero.damping_derivatives = struct();
aero.damping_derivatives.Clpm = Clpm;      % 롤 댐핑 미분계수
aero.damping_derivatives.Cmqm = Cmqm;      % 피치 댐핑 미분계수
aero.damping_derivatives.Cyawrm = Cyawrm;  % 요 댐핑 미분계수

% 댐핑 모멘트 계수
aero.damping_moments = struct();
aero.damping_moments.Clmd = Clmd;      % 롤 댐핑 모멘트 계수
aero.damping_moments.Cmmd = Cmmd;      % 피치 댐핑 모멘트 계수
aero.damping_moments.Cyawmd = Cyawmd;  % 요 댐핑 모멘트 계수

% 배열 형태로 저장 (빠른 접근용)
aero.coefficients = [CAm, CYm, CNm, Clm, Cmm, Cyawm, Clpm, Cmqm, Cyawrm, Clmd, Cmmd, Cyawmd];
aero.names = {'CAm', 'CYm', 'CNm', 'Clm', 'Cmm', 'Cyawm', 'Clpm', 'Cmqm', 'Cyawrm', 'Clmd', 'Cmmd', 'Cyawmd'};

% 메타데이터
aero.meta = struct();
aero.meta.alpha = alpha;
aero.meta.mach = mach;
aero.meta.angular_rates = [pm, qm, rm];
end

%% 정적 공력 계수 보간 함수
function [CAm, CYm, CNm, Clm, Cmm, Cyawm] = interpolateStaticCoef(aero_data, alpha, mach, alpha_range, mach_range)
% 정적 공력 계수 보간 함수

    % 가장 가까운 alpha와 mach 인덱스 찾기
    [~, alpha_idx] = min(abs(alpha_range - alpha));
    [~, mach_idx] = min(abs(mach_range - mach));
    
    % 해당하는 데이터에서 값 추출 (미래 개선: 2D 보간 구현)
    mask = (aero_data.AoA == alpha_range(alpha_idx)) & (aero_data.Mach == mach_range(mach_idx));
    
    if any(mask)
        row = aero_data(mask, :);
        CAm = row.CA(1);  % 축력 계수
        CNm = row.CN(1);  % 법선력 계수
        CYm = row.CY(1);  % 측력 계수
        
        if ismember('Clm', row.Properties.VariableNames)
            Clm = row.Clm(1); % 롤링 모멘트
        else
            Clm = 0;
        end
        
        if ismember('Cmm', row.Properties.VariableNames)
            Cmm = row.Cmm(1); % 피칭 모멘트
        else
            Cmm = 0;
        end
        
        if ismember('Cyawm', row.Properties.VariableNames)
            Cyawm = row.Cyawm(1); % 요잉 모멘트
        else
            Cyawm = 0;
        end
    else
        % 데이터가 없는 경우 기본값
        CAm = 0.5; CYm = 0; CNm = 0;
        Clm = 0; Cmm = 0; Cyawm = 0;
    end
end

% 로켓 댐핑 계수 계산 함수
function [Clpm, Cmqm, Cyawrm, Clmd, Cmmd, Cyawmd] = calcDampCoef(params, alpha, Mach, pm, qm, rm, aero_data, alpha_range, mach_range, cm_slopes)
% 로켓 댐핑 계수 계산 함수

    % 로켓 파라미터 추출
    Dref = params.vehicle.D_ref;      % 기준 길이 (로켓 직경) [m]
    
    % 음속 확인
    if isfield(params.environment, 'sound_speed')
        sound_speed = params.environment.sound_speed;
    else
        sound_speed = 340; % 기본 음속 [m/s]
    end
    
    VR = Mach * sound_speed;         % 상대속도 추정 (마하수 * 음속) [m/s]
    
    % 총 받음각 범위 보정 (0~180도)
    alpha = mod(alpha, 180);
    
    % 공력 데이터가 로드되었는지 확인
    if ~isempty(aero_data) && ~isempty(cm_slopes)
        % 로드된 공력 데이터 사용
        [Clpm, Cmqm, Cyawrm] = estimateDampingFromData(alpha, Mach, alpha_range, mach_range, cm_slopes);
    else
        % 공력 데이터 없음 - 경험적 모델 사용
        [Clpm, Cmqm, Cyawrm] = calcDampCoefEmpirical(params, alpha, Mach);
    end
    
    % 댐핑 모멘트 계수 계산
    Clmd = (pm * Dref / (2 * VR)) * Clpm;
    Cmmd = (qm * Dref / (2 * VR)) * Cmqm;
    Cyawmd = (rm * Dref / (2 * VR)) * Cyawrm;
end

% 공력 데이터에서 댐핑 계수 추정 함수
function [Clpm, Cmqm, Cyawrm] = estimateDampingFromData(alpha, Mach, alpha_range, mach_range, cm_slopes)
% 공력 데이터에서 댐핑 계수 추정

    % 가장 가까운 받음각과 마하수 인덱스 찾기
    [~, alpha_idx] = min(abs(alpha_range - alpha));
    [~, mach_idx] = min(abs(mach_range - Mach));
    
    % 1. 롤 댐핑 미분계수 (Clpm)
    % 롤 댐핑은 CFD로부터 직접 추정하기 어려움 - 경험적 공식 사용
    Clpm = calcRollDampEmpirical(alpha, Mach);
    
    % 2. 피치 댐핑 미분계수 (Cmqm)
    % 피치 댐핑은 Cm의 α 기울기와 관련됨
    Cmqm = cm_slopes.dCmm_dalpha(alpha_idx, mach_idx);
    
    % 댐핑 미분계수의 부호 보정 (물리적으로 의미 있는 값)
    % 댐핑 계수는 일반적으로 음수임 (안정화 역할)
    Cmqm = -abs(Cmqm);
    
    % 3. 요 댐핑 미분계수 (Cyawrm)
    % 낮은 받음각에서는 피치와 유사, 90도에서 크게 감소
    alpha_rad = alpha * pi/180;
    yaw_to_pitch_ratio = abs(cos(alpha_rad)) + 0.15;  % 0도에서 1.15, 90도에서 0.15
    
    % 요 댐핑 최종 계산
    if isfield(cm_slopes, 'dCyawm_dalpha')
        % 직접 계산된 요 기울기가 있으면 사용
        Cyawrm_raw = cm_slopes.dCyawm_dalpha(alpha_idx, mach_idx);
        Cyawrm = -abs(Cyawrm_raw);
    else
        % 아니면 피치로부터 추정
        Cyawrm = Cmqm * yaw_to_pitch_ratio;
    end
    
    % 댐핑 미분계수의 물리적 범위 제한
    Clpm = max(min(Clpm, 0), -2.0);   % -2.0 <= Clpm <= 0
    Cmqm = max(min(Cmqm, 0), -15.0);  % -15.0 <= Cmqm <= 0
    Cyawrm = max(min(Cyawrm, 0), -15.0);  % -15.0 <= Cyawrm <= 0
end

% 경험적 댐핑 계수 계산 함수
function [Clpm, Cmqm, Cyawrm] = calcDampCoefEmpirical(params, alpha, Mach)
% 경험적 모델을 통한 댐핑 계수 계산

    % 로켓 파라미터 추출
    Dref = params.vehicle.D_ref;      % 기준 길이 (로켓 직경) [m]
    L = params.vehicle.L;            % 로켓 길이 [m]
    fineness_ratio = L / Dref;       % 세장비
    
    % 1.1 롤 댐핑 미분계수 (Clpm)
    Clpm = calcRollDampEmpirical(alpha, Mach);
    
    % 1.2 피치 댐핑 미분계수 (Cmqm)
    % 세장비 효과
    L_D_factor = min(fineness_ratio / 10, 2.0);  % 세장비 스케일링 (최대 2.0)
    
    % 받음각 효과
    alpha_rad = alpha * pi/180;
    alpha_factor = (cos(alpha_rad)^2 + 0.2);  % 90도에서도 최소값 보장
    
    % 마하수 효과
    if Mach < 0.8
        % 아음속 영역
        mach_factor = 1.0;
    elseif Mach < 1.2
        % 천음속 영역 - 댐핑 증가
        mach_factor = 1.0 + 0.7 * (Mach - 0.8) / 0.4;
    else
        % 초음속 영역
        mach_factor = 1.7 - 0.2 * (Mach - 1.2);
        mach_factor = max(mach_factor, 1.0);  % 최소값 보장
    end
    
    % 기본 댐핑 계수
    base_Cmqm = -1.5 * L_D_factor;
    
    % 최종 피치 댐핑 미분계수
    Cmqm = base_Cmqm * alpha_factor * mach_factor;
    
    % 1.3 요 댐핑 미분계수 (Cyawrm)
    yaw_to_pitch_ratio = abs(cos(alpha_rad)) + 0.15;  % 0°에서 1.15, 90°에서 0.15
    Cyawrm = Cmqm * yaw_to_pitch_ratio;
end

% 롤 댐핑 미분계수 계산 함수 (경험적 모델)
function Clpm = calcRollDampEmpirical(alpha, Mach)
% 롤 댐핑 미분계수 계산 - 경험적 모델

    % 받음각에 따른 효과
    alpha_rad = alpha * pi/180;
    alpha_factor = cos(alpha_rad)^2;  % 0도에서 최대, 90도에서 최소
    
    % 마하수에 따른 효과 (경험적 모델)
    if Mach < 0.8
        % 아음속 영역
        mach_factor = 1.0;
    elseif Mach < 1.2
        % 천음속 영역 - 댐핑 증가
        mach_factor = 1.0 + 0.5 * (Mach - 0.8) / 0.4;
    else
        % 초음속 영역 - 댐핑 감소
        mach_factor = 1.5 - 0.1 * (Mach - 1.2);
        mach_factor = max(mach_factor, 0.9);  % 최소값 보장
    end
    
    % 기본 댐핑 계수
    base_Clpm = -0.3;
    
    % 최종 롤 댐핑 미분계수
    Clpm = base_Clpm * alpha_factor * mach_factor;
end

% 모멘트 계수 기울기 계산 함수
function slopes = calculateMomentSlopes(cfd_data, alpha_range, mach_range)
% 모멘트 계수의 알파 기울기 (dCm/dα) 및 관련 미분 계산

% 결과 저장 구조체 초기화
slopes = struct();
slopes.dCmm_dalpha = zeros(length(alpha_range), length(mach_range));

% 각 마하수에 대해 처리
for m = 1:length(mach_range)
    current_mach = mach_range(m);
    
    % 현재 마하수 데이터만 추출
    mach_mask = abs(cfd_data.Mach - current_mach) < 0.01;
    mach_data = cfd_data(mach_mask, :);
    
    % 받음각 순으로 정렬
    [sorted_alpha, sort_idx] = sort(mach_data.AoA);
    
    % 모멘트 계수 추출 (필드가 있는지 확인)
    if ismember('Cmm', mach_data.Properties.VariableNames)
        sorted_Cmm = mach_data.Cmm(sort_idx);
        
        % 모든 받음각 간격에 대한 기울기 계산
        if length(sorted_alpha) > 1
            % 중앙 차분법으로 기울기 계산
            dCmm = diff(sorted_Cmm);
            dAlpha = diff(sorted_alpha) * pi/180;  % 라디안 변환
            
            dCm_dAlpha = dCmm ./ dAlpha;
            
            % 각 alpha_range 값에 가장 가까운 기울기 찾기
            for a = 1:length(alpha_range)
                target_alpha = alpha_range(a);
                
                % 가장 가까운 데이터 지점 찾기
                [~, closest_idx] = min(abs(sorted_alpha - target_alpha));
                
                % 기울기 인덱스 (경계 체크)
                if closest_idx < length(sorted_alpha)
                    slope_idx = closest_idx;
                else
                    slope_idx = length(sorted_alpha) - 1;
                end
                
                % 기울기가 계산되었다면 저장
                if slope_idx > 0 && slope_idx <= length(dCm_dAlpha)
                    slopes.dCmm_dalpha(a, m) = dCm_dAlpha(slope_idx);
                else
                    % 기본값 (-1) 사용
                    slopes.dCmm_dalpha(a, m) = -1.0;
                end
            end
        else
            % 데이터 부족 - 기본값 설정
            slopes.dCmm_dalpha(:, m) = -1.0;
        end
    end
    
    % 요 모멘트 (Cyawm) 계수에 대해서도 동일한 처리
    if ismember('Cyawm', mach_data.Properties.VariableNames)
        sorted_Cyawm = mach_data.Cyawm(sort_idx);
        
        % Cyawm 필드 초기화
        if ~isfield(slopes, 'dCyawm_dalpha')
            slopes.dCyawm_dalpha = zeros(length(alpha_range), length(mach_range));
        end
        
        % 모든 받음각 간격에 대한 기울기 계산
        if length(sorted_alpha) > 1
            % 중앙 차분법으로 기울기 계산
            dCyawm = diff(sorted_Cyawm);
            dAlpha = diff(sorted_alpha) * pi/180;  % 라디안 변환
            
            dCyaw_dAlpha = dCyawm ./ dAlpha;
            
            % 각 alpha_range 값에 가장 가까운 기울기 찾기
            for a = 1:length(alpha_range)
                target_alpha = alpha_range(a);
                
                % 가장 가까운 데이터 지점 찾기
                [~, closest_idx] = min(abs(sorted_alpha - target_alpha));
                
                % 기울기 인덱스 (경계 체크)
                if closest_idx < length(sorted_alpha)
                    slope_idx = closest_idx;
                else
                    slope_idx = length(sorted_alpha) - 1;
                end
                
                % 기울기가 계산되었다면 저장
                if slope_idx > 0 && slope_idx <= length(dCyaw_dAlpha)
                    slopes.dCyawm_dalpha(a, m) = dCyaw_dAlpha(slope_idx);
                else
                    % 기본값 (-1) 사용
                    slopes.dCyawm_dalpha(a, m) = -1.0;
                end
            end
        else
            % 데이터 부족 - 기본값 설정
            slopes.dCyawm_dalpha(:, m) = -1.0;
        end
    end
end

% 댐핑 미분계수로 변환하기 위한 경험적 스케일링
pitch_scale = -10;  % dCm/dα → Cmq 변환 계수
yaw_scale = -10;    % dCyaw/dα → Cyawr 변환 계수

slopes.dCmm_dalpha = slopes.dCmm_dalpha * pitch_scale;

if isfield(slopes, 'dCyawm_dalpha')
    slopes.dCyawm_dalpha = slopes.dCyawm_dalpha * yaw_scale;
end

% 물리적으로 의미 있는 값으로 보정
fprintf('모멘트 기울기 계산 완료:\n');
fprintf('  피치 모멘트 기울기(dCmm/dα) 범위: [%.4f, %.4f]\n', ...
        min(slopes.dCmm_dalpha(:)), max(slopes.dCmm_dalpha(:)));

if isfield(slopes, 'dCyawm_dalpha')
    fprintf('  요 모멘트 기울기(dCyawm/dα) 범위: [%.4f, %.4f]\n', ...
            min(slopes.dCyawm_dalpha(:)), max(slopes.dCyawm_dalpha(:)));
end
end