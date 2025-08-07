function [Clpm, Cmqm, Cyawrm, Clmd, Cmmd, Cyawmd] = calcDampCoef(params, alpha, Mach, pm, qm, rm)
% calcDampCoef 로켓 댐핑 계수 계산 함수 - 미분 기반 접근법 우선 사용
%
% [Clpm, Cmqm, Cyawrm, Clmd, Cmmd, Cyawmd] = calcDampCoef(params, alpha, Mach, pm, qm, rm)
%
% 입력:
% params - vehicle_params 함수에서 생성된 로켓 파라미터 구조체
% alpha  - 총 받음각 [도]
% Mach   - 마하수
% pm     - 롤 각속도 [rad/s]
% qm     - 피치 각속도 [rad/s]
% rm     - 요 각속도 [rad/s]
%
% 출력:
% Clpm   - 롤 댐핑 미분계수
% Cmqm   - 피치 댐핑 미분계수
% Cyawrm - 요 댐핑 미분계수
% Clmd   - 롤 댐핑 모멘트 계수
% Cmmd   - 피치 댐핑 모멘트 계수
% Cyawmd - 요 댐핑 모멘트 계수

% 입력 유효성 검사
if nargin < 6
    error('calcDampCoef:입력부족', '6개 입력이 필요합니다: params, alpha, Mach, pm, qm, rm');
end

% 로켓 파라미터 추출
Dref = params.vehicle.D_ref;      % 기준 길이 (로켓 직경) [m]

% 음속 필드 확인 및 디폴트 값 설정
if isfield(params.environment, 'sound_speed')
    sound_speed = params.environment.sound_speed;
else
    sound_speed = 340; % 기본 음속 [m/s]
end

VR = Mach * sound_speed;         % 상대속도 추정 (마하수 * 음속) [m/s]

% 총 받음각 범위 보정 (0~180도)
alpha = mod(alpha, 180);

% 1. CFD 데이터 로드 (첫 호출시만)
persistent cfd_data alpha_range mach_range
persistent moment_slopes roll_data_available

if isempty(cfd_data)
    % CSV 파일 경로 설정
    csv_path = 'csv_Force_moment_arg.csv';
    
    try
        % 옵션 설정 - 열 이름 보존
        opts = detectImportOptions(csv_path, 'VariableNamingRule', 'preserve');
        cfd_data = readtable(csv_path, opts);
        
        % 유니크한 받음각과 마하수 추출
        alpha_range = unique(cfd_data.AoA);
        mach_range = unique(cfd_data.Mach);
        
        fprintf('CFD 데이터 로드 완료: %d개 받음각, %d개 마하수\n', ...
                length(alpha_range), length(mach_range));
        
        % 기울기(미분계수) 미리 계산
        moment_slopes = calculateMomentSlopes(cfd_data, alpha_range, mach_range);
        
        % 롤링 데이터 가용성 확인
        roll_data_available = isfield(moment_slopes, 'dClm_dp') && ...
                             ~isempty(moment_slopes.dClm_dp) && ...
                             any(moment_slopes.dClm_dp(:) ~= 0);
        
        if roll_data_available
            fprintf('롤 댐핑 미분 데이터 사용 가능\n');
        else
            fprintf('롤 댐핑 미분 데이터 없음 - 경험적 모델 사용\n');
        end
        
    catch ME
        warning('CFD 데이터 로드 실패: %s\n경험적 모델로 대체합니다.', E.message);
        cfd_data = [];
        roll_data_available = false;
    end
end

% 2. 댐핑 미분계수 계산
% CFD 데이터가 없는 경우 경험적 모델 사용
if isempty(cfd_data)
    [Clpm, Cmqm, Cyawrm, Clmd, Cmmd, Cyawmd] = calcDampCoefEmpirical(params, alpha, Mach, pm, qm, rm);
    return;
end

% CFD 데이터로부터 댐핑 계수 추정
[Clpm, Cmqm, Cyawrm] = estimateDampingFromCFD(alpha, Mach, moment_slopes, alpha_range, mach_range, roll_data_available);

% 3. 댐핑 모멘트 계수 계산
% 문서의 방정식 7,8,9 사용
Clmd = (pm * Dref / (2 * VR)) * Clpm;
Cmmd = (qm * Dref / (2 * VR)) * Cmqm;
Cyawmd = (rm * Dref / (2 * VR)) * Cyawrm;

end

%% CFD 데이터로부터 댐핑 계수 추정 함수
function [Clpm, Cmqm, Cyawrm] = estimateDampingFromCFD(alpha, Mach, moment_slopes, alpha_range, mach_range, roll_data_available)
% CFD 데이터에서 정적 모멘트 계수의 기울기를 이용하여 댐핑 계수 추정

% 가장 가까운 받음각과 마하수 인덱스 찾기
[~, alpha_idx] = min(abs(alpha_range - alpha));
[~, mach_idx] = min(abs(mach_range - Mach));

% 1. 롤 댐핑 미분계수 (Clpm)
% 우선 미분 데이터를 사용하고, 없을 경우 경험적 모델 사용
if roll_data_available
    % 미분 데이터가 있으면 사용
    Clpm_raw = moment_slopes.dClm_dp(alpha_idx, mach_idx);
    Clpm = Clpm_raw; % 미분값 그대로 사용 (이미 스케일링 완료)
else
    % 미분 데이터가 없으면 경험적 모델 사용
    Clpm = calcRollDampEmpirical(alpha, Mach);
end

% 2. 피치 댐핑 미분계수 (Cmqm)
% 피치 댐핑은 Cm의 α 기울기와 관련됨
Cmqm = moment_slopes.dCmm_dalpha(alpha_idx, mach_idx);

% 댐핑 미분계수의 부호 보정 (물리적으로 의미 있는 값)
% 댐핑 계수는 일반적으로 음수임 (안정화 역할)
Cmqm = -abs(Cmqm);

% 3. 요 댐핑 미분계수 (Cyawrm)
% 낮은 받음각에서는 피치와 유사, 90도에서 크게 감소
alpha_rad = alpha * pi/180;
yaw_to_pitch_ratio = abs(cos(alpha_rad)) + 0.15;  % 0도에서 1.15, 90도에서 0.15

% 요 댐핑 최종 계산
if isfield(moment_slopes, 'dCyawm_dalpha')
    % 직접 계산된 요 기울기가 있으면 사용
    Cyawrm_raw = moment_slopes.dCyawm_dalpha(alpha_idx, mach_idx);
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

%% 모멘트 계수 기울기 계산 함수
function slopes = calculateMomentSlopes(cfd_data, alpha_range, mach_range)
% 모멘트 계수의 미분값 계산 - 알파, 롤 각속도 등에 대한 미분 포함

% 결과 저장 구조체 초기화
slopes = struct();
slopes.dCmm_dalpha = zeros(length(alpha_range), length(mach_range));

% 롤링 모멘트 미분 필드 초기화 (데이터 가용성 확인 필요)
if ismember('Clm', cfd_data.Properties.VariableNames)
    slopes.dClm_dp = zeros(length(alpha_range), length(mach_range));
    fprintf('롤링 모멘트 필드 있음 - 미분 계산 시도\n');
else
    fprintf('롤링 모멘트 필드 없음 - 롤 댐핑은 경험적 모델 사용\n');
end

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
    
    % 롤링 모멘트 (Clm) 계수에 대한 미분 계산 - 확장된 부분
    if ismember('Clm', mach_data.Properties.VariableNames) && isfield(slopes, 'dClm_dp')
        sorted_Clm = mach_data.Clm(sort_idx);
        
        % 각 받음각에 대해 롤 속도에 따른 미분 추정
        % 참고: 일반적으로 CFD 데이터는 다양한 롤 각속도에 대한 데이터를 포함하지 않을 수 있음
        % 이 경우 이론적 기울기 추정 또는 경험적인 관계식 사용 
        
        % 예: 기본적인 추정 - 롤링 모멘트를 alpha와 Mach의 함수로 가정
        if length(sorted_alpha) > 1
            % 예시: alpha 변화에 따른 Clm 변화로부터 미분 추정
            dClm = diff(sorted_Clm);
            dAlpha = diff(sorted_alpha) * pi/180;  % 라디안 변환
            
            % 단순화된 추정 (향후 개선 필요)
            dClm_dp_estimate = -abs(dClm ./ dAlpha) * 0.5;  % 스케일링 팩터 적용
            
            % 각 alpha_range 값에 가장 가까운 미분값 찾기
            for a = 1:length(alpha_range)
                target_alpha = alpha_range(a);
                
                % 가장 가까운 데이터 지점 찾기
                [~, closest_idx] = min(abs(sorted_alpha - target_alpha));
                
                % 미분 인덱스 (경계 체크)
                if closest_idx < length(sorted_alpha)
                    derivative_idx = closest_idx;
                else
                    derivative_idx = length(sorted_alpha) - 1;
                end
                
                % 미분값이 계산되었다면 저장
                if derivative_idx > 0 && derivative_idx <= length(dClm_dp_estimate)
                    % 받음각에 따른 보정 (높은 받음각에서 효과 감소)
                    alpha_factor = cos(target_alpha * pi/180)^2;
                    slopes.dClm_dp(a, m) = dClm_dp_estimate(derivative_idx) * alpha_factor;
                else
                    % 기본값 (-0.3) 사용
                    slopes.dClm_dp(a, m) = -0.3;
                end
            end
        else
            % 데이터 부족 - 기본값 설정
            slopes.dClm_dp(:, m) = -0.3;
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

% 기울기 크기 조정 (스케일링 계수)
% 댐핑 미분계수로 변환하기 위한 경험적 스케일링
pitch_scale = -2.0;  % dCm/dα → Cmq 변환 계수
yaw_scale = -2.0;    % dCyaw/dα → Cyawr 변환 계수
roll_scale = -1.0;   % dClm/dp → Clp 변환 계수 (추가됨)

slopes.dCmm_dalpha = slopes.dCmm_dalpha * pitch_scale;

if isfield(slopes, 'dCyawm_dalpha')
    slopes.dCyawm_dalpha = slopes.dCyawm_dalpha * yaw_scale;
end

if isfield(slopes, 'dClm_dp')
    slopes.dClm_dp = slopes.dClm_dp * roll_scale;
end

% 물리적으로 의미 있는 값으로 보정
fprintf('모멘트 기울기 계산 완료:\n');
fprintf('  피치 모멘트 기울기(dCmm/dα) 범위: [%.4f, %.4f]\n', ...
        min(slopes.dCmm_dalpha(:)), max(slopes.dCmm_dalpha(:)));

if isfield(slopes, 'dCyawm_dalpha')
    fprintf('  요 모멘트 기울기(dCyawm/dα) 범위: [%.4f, %.4f]\n', ...
            min(slopes.dCyawm_dalpha(:)), max(slopes.dCyawm_dalpha(:)));
end

if isfield(slopes, 'dClm_dp')
    fprintf('  롤 모멘트 기울기(dClm/dp) 범위: [%.4f, %.4f]\n', ...
            min(slopes.dClm_dp(:)), max(slopes.dClm_dp(:)));
end
end

%% 경험적 모델 함수들 (미분 기반 방법이 불가능할 경우 대체용)
function [Clpm, Cmqm, Cyawrm, Clmd, Cmmd, Cyawmd] = calcDampCoefEmpirical(params, alpha, Mach, pm, qm, rm)
% 경험적 모델을 사용한 댐핑 계수 계산 (CFD 데이터 없을 경우 사용)

% 로켓 파라미터 추출
Dref = params.vehicle.D_ref;      % 기준 길이 (로켓 직경) [m]
L = params.vehicle.L;            % 로켓 길이 [m]
fineness_ratio = L / Dref;       % 세장비

% 음속 필드 확인 및 디폴트 값 설정
if isfield(params.environment, 'sound_speed')
    sound_speed = params.environment.sound_speed;
else
    sound_speed = 340; % 기본 음속 [m/s]
end

VR = Mach * sound_speed;         % 상대속도 추정 (마하수 * 음속) [m/s]

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

% 2. 댐핑 모멘트 계수
Clmd = (pm * Dref / (2 * VR)) * Clpm;
Cmmd = (qm * Dref / (2 * VR)) * Cmqm;
Cyawmd = (rm * Dref / (2 * VR)) * Cyawrm;
end

%%
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
disp('경험적 모델 사용 Clpm')
end