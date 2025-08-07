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
% aero : 공력 계수가 포함된 구조체

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
        % 파일 경로 설정
        force_moment_path = 'csv_Force_moment_axiseq.csv';  % 공력계수 CSV 파일
        opts = detectImportOptions(force_moment_path, 'VariableNamingRule', 'preserve');
        aero_data = readtable(force_moment_path, opts);

        % X축 관련 계수들의 부호 반전 - 좌표계 변환(CFD 해석진행 X축 방향 반대임)
        aero_data.CA = -aero_data.CA;     % 축력 계수 반전
        aero_data.Clm = -aero_data.Clm;   % 롤링 모멘트 계수 반전
        aero_data.CN = -aero_data.CN;     % 법선력 계수 반전
        aero_data.Cmm = -aero_data.Cmm;   % 피칭 모멘트 계수 반전
        aero_data.Cyawm = -aero_data.Cyawm; % 요잉 모멘트 계수 반전
        
        % 받음각과 마하수 범위 추출
        alpha_range = unique(aero_data.AoA);
        mach_range = unique(aero_data.Mach);
        
        % 기울기(미분계수) 미리 계산
        cm_slopes = calculateMomentSlopes(aero_data, alpha_range, mach_range);
        
        fprintf('공력 데이터 로드 완료: %d개 받음각, %d개 마하수\n', ...
                length(alpha_range), length(mach_range));
    catch ME
        warning('공력 데이터 로드 실패: %s\n경험적 모델로 대체합니다.', ME.message);
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

% 속도 관련 미분계수 계산 (개선된 버전)
[CXu, CYv, CZw, Clp, Cmq, Cnr] = calcVelocityDerivatives(params, alpha, mach, CAm, CYm, CNm, Clpm, Cmqm, Cyawrm, aero_data, alpha_range, mach_range);

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

% 속도 미분계수 (자코비안 행렬용)
aero.velocity_derivatives = struct();
aero.velocity_derivatives.CXu = CXu;       % X방향 속도 변화에 대한 축력 미분계수
aero.velocity_derivatives.CYv = CYv;       % Y방향 속도 변화에 대한 측력 미분계수
aero.velocity_derivatives.CZw = CZw;       % Z방향 속도 변화에 대한 법선력 미분계수
aero.velocity_derivatives.Clp = Clp;       % 롤 속도 변화에 대한 롤 모멘트 미분계수
aero.velocity_derivatives.Cmq = Cmq;       % 피치 속도 변화에 대한 피치 모멘트 미분계수
aero.velocity_derivatives.Cnr = Cnr;       % 요 속도 변화에 대한 요 모멘트 미분계수

% 배열 형태로 저장 (빠른 접근용)
aero.coefficients = [CAm, CYm, CNm, Clm, Cmm, Cyawm, Clpm, Cmqm, Cyawrm, Clmd, Cmmd, Cyawmd, ...
                    CXu, CYv, CZw, Clp, Cmq, Cnr];  % 속도 미분계수 포함하도록 확장
aero.names = {'CAm', 'CYm', 'CNm', 'Clm', 'Cmm', 'Cyawm', 'Clpm', 'Cmqm', 'Cyawrm', 'Clmd', 'Cmmd', 'Cyawmd', ...
              'CXu', 'CYv', 'CZw', 'Clp', 'Cmq', 'Cnr'};  % 이름 확장

% 메타데이터
aero.meta = struct();
aero.meta.alpha = alpha;
aero.meta.mach = mach;
aero.meta.angular_rates = [pm, qm, rm];
end

%% 정적 공력 계수 보간 함수
function [CAm, CYm, CNm, Clm, Cmm, Cyawm] = interpolateStaticCoef(aero_data, alpha, mach, alpha_range, mach_range)
    % 2D 선형 보간법을 사용한 정적 공력 계수 계산 함수
    
    % 결과값 초기화
    CAm = 0.5; CYm = 0; CNm = 0; Clm = 0; Cmm = 0; Cyawm = 0;
    
    % 데이터 범위 확인
    if alpha < min(alpha_range) || alpha > max(alpha_range) || ...
       mach < min(mach_range) || mach > max(mach_range)
        % 범위를 벗어나면 경계값으로 제한
        alpha = max(min(alpha, max(alpha_range)), min(alpha_range));
        mach = max(min(mach, max(mach_range)), min(mach_range));
    end
    
    % 주변 데이터 포인트 찾기
    alpha_idx_low = find(alpha_range <= alpha, 1, 'last');
    alpha_idx_high = find(alpha_range > alpha, 1, 'first');
    
    mach_idx_low = find(mach_range <= mach, 1, 'last');
    mach_idx_high = find(mach_range > mach, 1, 'first');
    
    % 경계 처리
    if isempty(alpha_idx_low), alpha_idx_low = 1; end
    if isempty(alpha_idx_high), alpha_idx_high = length(alpha_range); end
    if isempty(mach_idx_low), mach_idx_low = 1; end
    if isempty(mach_idx_high), mach_idx_high = length(mach_range); end
    
    % 일치하는 경우(보간 불필요)
    if alpha_idx_low == alpha_idx_high && mach_idx_low == mach_idx_high
        % 정확히 일치하는 데이터 포인트 찾기
        mask = (aero_data.AoA == alpha_range(alpha_idx_low)) & ...
               (aero_data.Mach == mach_range(mach_idx_low));
        
        if any(mask)
            row = aero_data(mask, :);
            CAm = row.CA(1);
            CYm = row.CY(1);
            CNm = row.CN(1);
            
            if ismember('Clm', row.Properties.VariableNames)
                Clm = row.Clm(1);
            end
            
            if ismember('Cmm', row.Properties.VariableNames)
                Cmm = row.Cmm(1);
            end
            
            if ismember('Cyawm', row.Properties.VariableNames)
                Cyawm = row.Cyawm(1);
            end
        end
        return;
    end
    
    % 2D 선형 보간에 필요한 4개 데이터 포인트 값 가져오기
    alpha_low = alpha_range(alpha_idx_low);
    alpha_high = alpha_range(alpha_idx_high);
    mach_low = mach_range(mach_idx_low);
    mach_high = mach_range(mach_idx_high);
    
    % 4개의 격자점 데이터
    CA_points = zeros(2, 2);
    CY_points = zeros(2, 2);
    CN_points = zeros(2, 2);
    Clm_points = zeros(2, 2);
    Cmm_points = zeros(2, 2);
    Cyawm_points = zeros(2, 2);
    
    % 격자점 데이터 추출
    for i = 1:2
        for j = 1:2
            if i == 1
                curr_alpha = alpha_low;
                alpha_idx = alpha_idx_low;
            else
                curr_alpha = alpha_high;
                alpha_idx = alpha_idx_high;
            end
            
            if j == 1
                curr_mach = mach_low;
                mach_idx = mach_idx_low;
            else
                curr_mach = mach_high;
                mach_idx = mach_idx_high;
            end
            
            % 해당 격자점 데이터 찾기
            mask = (aero_data.AoA == alpha_range(alpha_idx)) & ...
                   (aero_data.Mach == mach_range(mach_idx));
            
            if any(mask)
                row = aero_data(mask, :);
                CA_points(i, j) = row.CA(1);
                CY_points(i, j) = row.CY(1);
                CN_points(i, j) = row.CN(1);
                
                if ismember('Clm', row.Properties.VariableNames)
                    Clm_points(i, j) = row.Clm(1);
                end
                
                if ismember('Cmm', row.Properties.VariableNames)
                    Cmm_points(i, j) = row.Cmm(1);
                end
                
                if ismember('Cyawm', row.Properties.VariableNames)
                    Cyawm_points(i, j) = row.Cyawm(1);
                end
            end
        end
    end
    
    % 보간 가중치 계산
    if alpha_high ~= alpha_low
        alpha_weight = (alpha - alpha_low) / (alpha_high - alpha_low);
    else
        alpha_weight = 0;
    end
    
    if mach_high ~= mach_low
        mach_weight = (mach - mach_low) / (mach_high - mach_low);
    else
        mach_weight = 0;
    end
    
    % 바이리니어 보간 수행
    % 먼저 알파 방향으로 보간
    CA_temp1 = CA_points(1, 1) + alpha_weight * (CA_points(2, 1) - CA_points(1, 1));
    CA_temp2 = CA_points(1, 2) + alpha_weight * (CA_points(2, 2) - CA_points(1, 2));
    % 다음 마하수 방향으로 보간
    CAm = CA_temp1 + mach_weight * (CA_temp2 - CA_temp1);
    
    % 나머지 계수들에 대해서도 동일한 방식으로 보간
    CY_temp1 = CY_points(1, 1) + alpha_weight * (CY_points(2, 1) - CY_points(1, 1));
    CY_temp2 = CY_points(1, 2) + alpha_weight * (CY_points(2, 2) - CY_points(1, 2));
    CYm = CY_temp1 + mach_weight * (CY_temp2 - CY_temp1);
    
    CN_temp1 = CN_points(1, 1) + alpha_weight * (CN_points(2, 1) - CN_points(1, 1));
    CN_temp2 = CN_points(1, 2) + alpha_weight * (CN_points(2, 2) - CN_points(1, 2));
    CNm = CN_temp1 + mach_weight * (CN_temp2 - CN_temp1);
    
    Clm_temp1 = Clm_points(1, 1) + alpha_weight * (Clm_points(2, 1) - Clm_points(1, 1));
    Clm_temp2 = Clm_points(1, 2) + alpha_weight * (Clm_points(2, 2) - Clm_points(1, 2));
    Clm = Clm_temp1 + mach_weight * (Clm_temp2 - Clm_temp1);
    
    Cmm_temp1 = Cmm_points(1, 1) + alpha_weight * (Cmm_points(2, 1) - Cmm_points(1, 1));
    Cmm_temp2 = Cmm_points(1, 2) + alpha_weight * (Cmm_points(2, 2) - Cmm_points(1, 2));
    Cmm = Cmm_temp1 + mach_weight * (Cmm_temp2 - Cmm_temp1);
    
    Cyawm_temp1 = Cyawm_points(1, 1) + alpha_weight * (Cyawm_points(2, 1) - Cyawm_points(1, 1));
    Cyawm_temp2 = Cyawm_points(1, 2) + alpha_weight * (Cyawm_points(2, 2) - Cyawm_points(1, 2));
    Cyawm = Cyawm_temp1 + mach_weight * (Cyawm_temp2 - Cyawm_temp1);
    
    % NaN 값 체크 및 대체
    if isnan(CAm), CAm = 0.5; end
    if isnan(CYm), CYm = 0; end
    if isnan(CNm), CNm = 0; end
    if isnan(Clm), Clm = 0; end
    if isnan(Cmm), Cmm = 0; end
    if isnan(Cyawm), Cyawm = 0; end
end

% 로켓 댐핑 계수 계산 함수
function [Clpm, Cmqm, Cyawrm, Clmd, Cmmd, Cyawmd] = calcDampCoef(params, alpha, Mach, pm, qm, rm, aero_data, alpha_range, mach_range, cm_slopes)
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
    % 결과 저장 구조체 초기화
    slopes = struct();
    slopes.dCmm_dalpha = zeros(length(alpha_range), length(mach_range));
    slopes.dCyawm_dalpha = zeros(length(alpha_range), length(mach_range));

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
                % 유효한 데이터 점 확인 (NaN 제외)
                valid_idx = ~isnan(sorted_Cmm);
                valid_alpha = sorted_alpha(valid_idx);
                valid_Cmm = sorted_Cmm(valid_idx);
                
                % 데이터가 충분한지 확인
                if length(valid_alpha) > 1
                    % 중앙 차분법으로 기울기 계산
                    dCmm = diff(valid_Cmm);
                    dAlpha = diff(valid_alpha) * pi/180;  % 라디안 변환
                    
                    % 비정상적으로 큰 변화 필터링 (불연속 방지)
                    threshold = 5 * median(abs(dCmm ./ dAlpha));  % 중앙값의 5배를 임계값으로 설정
                    valid_slope_idx = abs(dCmm ./ dAlpha) < threshold;
                    
                    if any(valid_slope_idx)
                        dCm_dAlpha = dCmm(valid_slope_idx) ./ dAlpha(valid_slope_idx);
                        slope_alpha = valid_alpha(1:end-1) + diff(valid_alpha)/2;  % 중간점
                        slope_alpha = slope_alpha(valid_slope_idx);
                        
                        % 각 alpha_range 값에 대한 기울기 보간 또는 가장 가까운 값 사용
                        for a = 1:length(alpha_range)
                            target_alpha = alpha_range(a);
                            
                            % 보간 범위 내에 있는지 확인
                            if target_alpha >= min(slope_alpha) && target_alpha <= max(slope_alpha) && length(slope_alpha) > 1
                                % 선형 보간
                                slopes.dCmm_dalpha(a, m) = interp1(slope_alpha, dCm_dAlpha, target_alpha, 'linear');
                            else
                                % 가장 가까운 값 사용
                                [~, closest_idx] = min(abs(slope_alpha - target_alpha));
                                if ~isempty(closest_idx) && closest_idx <= length(dCm_dAlpha)
                                    slopes.dCmm_dalpha(a, m) = dCm_dAlpha(closest_idx);
                                else
                                    slopes.dCmm_dalpha(a, m) = -1.0;  % 기본값
                                end
                            end
                        end
                    else
                        % 유효한 기울기가 없는 경우
                        slopes.dCmm_dalpha(:, m) = -1.0;
                    end
                else
                    % 데이터 부족
                    slopes.dCmm_dalpha(:, m) = -1.0;
                end
            else
                % 데이터 부족
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
                % 유효한 데이터 점 확인 (NaN 제외)
                valid_idx = ~isnan(sorted_Cyawm);
                valid_alpha = sorted_alpha(valid_idx);
                valid_Cyawm = sorted_Cyawm(valid_idx);
                
                % 데이터가 충분한지 확인
                if length(valid_alpha) > 1
                    % 중앙 차분법으로 기울기 계산
                    dCyawm = diff(valid_Cyawm);
                    dAlpha = diff(valid_alpha) * pi/180;  % 라디안 변환
                    
                    % 비정상적으로 큰 변화 필터링 (불연속 방지)
                    threshold = 5 * median(abs(dCyawm ./ dAlpha));  % 중앙값의 5배를 임계값으로 설정
                    valid_slope_idx = abs(dCyawm ./ dAlpha) < threshold;
                    
                    if any(valid_slope_idx)
                        dCyaw_dAlpha = dCyawm(valid_slope_idx) ./ dAlpha(valid_slope_idx);
                        slope_alpha = valid_alpha(1:end-1) + diff(valid_alpha)/2;  % 중간점
                        slope_alpha = slope_alpha(valid_slope_idx);
                        
                        % 각 alpha_range 값에 대한 기울기 보간 또는 가장 가까운 값 사용
                        for a = 1:length(alpha_range)
                            target_alpha = alpha_range(a);
                            
                            % 보간 범위 내에 있는지 확인
                            if target_alpha >= min(slope_alpha) && target_alpha <= max(slope_alpha) && length(slope_alpha) > 1
                                % 선형 보간
                                slopes.dCyawm_dalpha(a, m) = interp1(slope_alpha, dCyaw_dAlpha, target_alpha, 'linear');
                            else
                                % 가장 가까운 값 사용
                                [~, closest_idx] = min(abs(slope_alpha - target_alpha));
                                if ~isempty(closest_idx) && closest_idx <= length(dCyaw_dAlpha)
                                    slopes.dCyawm_dalpha(a, m) = dCyaw_dAlpha(closest_idx);
                                else
                                    slopes.dCyawm_dalpha(a, m) = -1.0;  % 기본값
                                end
                            end
                        end
                    else
                        % 유효한 기울기가 없는 경우
                        slopes.dCyawm_dalpha(:, m) = -1.0;
                    end
                else
                    % 데이터 부족
                    slopes.dCyawm_dalpha(:, m) = -1.0;
                end
            else
                % 데이터 부족
                slopes.dCyawm_dalpha(:, m) = -1.0;
            end
        end
    end

    % 댐핑 미분계수로 변환하기 위한 경험적 스케일링
    pitch_scale = -12;  % dCm/dα → Cmqm 변환 계수
    yaw_scale = -12;    % dCyaw/dα → Cyawrm 변환 계수

    % 댐핑 계수 부호 검증 (물리적으로 타당한 방향으로)
    slopes.dCmm_dalpha = slopes.dCmm_dalpha * pitch_scale;
    slopes.dCmm_dalpha = -abs(slopes.dCmm_dalpha);  % 항상 음수 보장

    if isfield(slopes, 'dCyawm_dalpha')
        slopes.dCyawm_dalpha = slopes.dCyawm_dalpha * yaw_scale;
        slopes.dCyawm_dalpha = -abs(slopes.dCyawm_dalpha);  % 항상 음수 보장
    end

    % 댐핑 미분계수 범위 제한 
    slopes.dCmm_dalpha = max(min(slopes.dCmm_dalpha, 0), -15.0);  % -15.0 <= Cmqm <= 0
    if isfield(slopes, 'dCyawm_dalpha')
        slopes.dCyawm_dalpha = max(min(slopes.dCyawm_dalpha, 0), -15.0);  % -15.0 <= Cyawrm <= 0
    end

    % 디버깅 정보 출력
    fprintf('모멘트 기울기 계산 완료:\n');
    fprintf('  피치 모멘트 기울기(dCmm/dα) 범위: [%.4f, %.4f]\n', ...
            min(slopes.dCmm_dalpha(:)), max(slopes.dCmm_dalpha(:)));

    if isfield(slopes, 'dCyawm_dalpha')
        fprintf('  요 모멘트 기울기(dCyawm/dα) 범위: [%.4f, %.4f]\n', ...
                min(slopes.dCyawm_dalpha(:)), max(slopes.dCyawm_dalpha(:)));
    end
end

% 속도 미분계수 계산 함수 (개선된 버전)
function [CXu, CYv, CZw, Clp, Cmq, Cnr] = calcVelocityDerivatives(params, alpha_deg, mach, CAm, CYm, CNm, Clpm, Cmqm, Cyawrm, aero_data, alpha_range, mach_range)
    % 댐핑 계수는 이미 전달받은 값 사용
    Clp = Clpm;
    Cmq = Cmqm;
    Cnr = Cyawrm;
    
    % 미분 계산을 위한 마하수 변화량
    delta_mach = 0.01 * max(0.1, mach);  % 최소 0.001 이상
    
    % 음속 계산
    if isfield(params.environment, 'sound_speed')
        sound_speed = params.environment.sound_speed;
    else
        sound_speed = 340; % 기본 음속 [m/s]
    end
    
    % 마하수가 매우 작은 경우 기본값 사용
    if mach < 0.05
        CXu = 0;
        CYv = 0;
        CZw = 0;
    else
        % 마하수 범위 내에서 유효한 값 확인
        mach_plus = min(mach + delta_mach, max(mach_range));
        mach_minus = max(mach - delta_mach, min(mach_range));
        
        % 중앙 차분법을 사용한 미분계수 계산
        try
            % M+dM에서의 값 계산
            [CAm_plus, CYm_plus, CNm_plus, ~, ~, ~] = interpolateStaticCoef(aero_data, alpha_deg, mach_plus, alpha_range, mach_range);
            
            % M-dM에서의 값 계산
            [CAm_minus, CYm_minus, CNm_minus, ~, ~, ~] = interpolateStaticCoef(aero_data, alpha_deg, mach_minus, alpha_range, mach_range);
            
            % 미분계수 계산 (중앙 차분법)
            dCA_dM = (CAm_plus - CAm_minus) / (mach_plus - mach_minus);
            dCY_dM = (CYm_plus - CYm_minus) / (mach_plus - mach_minus);
            dCN_dM = (CNm_plus - CNm_minus) / (mach_plus - mach_minus);
            
            % 미분계수를 속도 기준으로 변환 (dC/dV = dC/dM * dM/dV)
            CXu = dCA_dM / sound_speed;
            CYv = dCY_dM / sound_speed;
            CZw = dCN_dM / sound_speed;
        catch
            % 보간에 실패한 경우 기본값 사용
            CXu = -0.02;
            CYv = -0.02;
            CZw = -0.02;
        end
    end
    
    % 물리적 타당성 확인 (항력 계수는 일반적으로 음수)
    if CXu > 0
        CXu = -abs(CXu);
    end
    
    if CYv > 0
        CYv = -abs(CYv);
    end
    
    if CZw > 0
        CZw = -abs(CZw);
    end
    
    % 받음각에 따른 조정
    if alpha_deg > 15
        alpha_factor = 1.0 + 0.05 * (alpha_deg - 15);
        alpha_factor = min(alpha_factor, 2.0);  % 최대 2배까지만 조정
        
        CXu = CXu * alpha_factor;
        CYv = CYv * alpha_factor;
        CZw = CZw * alpha_factor;
    end
    
    % 마하수 범위에 따른 추가 보정
    if mach > 1.0
        % 초음속 영역
        damping_scale = 0.8;
        Clp = Clp * damping_scale;
        Cmq = Cmq * damping_scale;
        Cnr = Cnr * damping_scale;
    elseif mach > 0.8
        % 천음속 영역
        CXu = CXu * 1.2; % 천음속 영역에서 변화 강화
        Cmq = Cmq * 0.7;
        Cnr = Cnr * 0.7;
    end
    
    % NaN 값 방지
    if isnan(CXu), CXu = -0.02; end
    if isnan(CYv), CYv = -0.02; end
    if isnan(CZw), CZw = -0.02; end
end