function [static_coeffs, damping_coeffs, damping_moments] = calculate_aero_coeffs(aero_data, alpha, mach, vel, omega_b)
% calculate_aero_coeffs  현재 비행 조건에서의 공력계수 계산 (B 프레임)
%
% [static_coeffs, damping_coeffs, damping_moments] = calculate_aero_coeffs(aero_data, alpha, mach, vel, omega_b)
%
% 입력:
% aero_data : load_aero_data 함수로부터 로드된 공력 데이터 구조체
% alpha : 받음각 [도]
% mach : 마하수
% vel : 속도 [m/s]
% omega_b : 각속도 벡터 [p;q;r] [rad/s]
%
% 출력:
% static_coeffs : 정적 공력계수 구조체 (B 프레임)
% damping_coeffs : 감쇠 도함수 구조체
% damping_moments : 감쇠 모멘트 계수 구조체

% 입력 검증
if nargin < 5
    error('calculate_aero_coeffs:NotEnoughInputs', '입력이 부족합니다.');
end

% 정적 계수 데이터와 감쇠 계수 데이터 추출
static_data = aero_data.static_coeffs.data;
alpha_arr = aero_data.alpha_arr;
mach_arr = aero_data.mach_arr;
vel_arr = aero_data.velocity_arr;

% 범위 검사 및 보정
alpha = max(min(alpha, max(alpha_arr)), min(alpha_arr));
mach = max(min(mach, max(mach_arr)), min(mach_arr));
vel = max(min(vel, max(vel_arr)), min(vel_arr));

% 1. 정적 계수(Static Coefficients) 보간
% 가장 가까운 속도 찾기
[~, vel_idx] = min(abs(vel_arr - vel));
vel_closest = vel_arr(vel_idx);

% 해당 속도의 데이터 필터링
vel_data = static_data(static_data.Velocity == vel_closest, :);

% 마하수와 받음각으로 보간
mach_arr_vel = unique(vel_data.Mach);
alpha_arr_vel = unique(vel_data.AoA);

% 마하수와 받음각 인덱스 찾기 (가장 가까운 값)
[~, mach_idx] = min(abs(mach_arr_vel - mach));
[~, alpha_idx] = min(abs(alpha_arr_vel - alpha));

mach_nearest = mach_arr_vel(mach_idx);
alpha_nearest = alpha_arr_vel(alpha_idx);

% 해당 마하수와 받음각의 데이터 찾기
row_idx = find(vel_data.Mach == mach_nearest & vel_data.AoA == alpha_nearest);

if isempty(row_idx)
    error('calculate_aero_coeffs:DataNotFound', ...
          '요청한 조건(Mach=%.2f, AoA=%d, V=%.1f)에 대한 데이터를 찾을 수 없습니다.', ...
          mach, alpha, vel);
end

% 정적 계수 추출 (B 프레임)
static_coeffs = struct();
static_coeffs.CA = vel_data.CA(row_idx(1));
static_coeffs.CY = vel_data.CY(row_idx(1));
static_coeffs.CN = vel_data.CN(row_idx(1));
static_coeffs.Cl = vel_data.Cl(row_idx(1));
static_coeffs.Cm = vel_data.Cm(row_idx(1));
static_coeffs.Cn = vel_data.Cn(row_idx(1));

% 2. 감쇠 계수(Damping Derivatives) 계산
damping_coeffs = struct();
damping_moments = struct();

% 감쇠 계수 데이터가 있는 경우
if isfield(aero_data.damping_derivs, 'data') && ~isempty(aero_data.damping_derivs.data)
    damping_data = aero_data.damping_derivs.data;
    
    % 가장 가까운 마하수, 받음각 찾기
    if ismember('Mach', damping_data.Properties.VariableNames) && ...
       ismember('AoA', damping_data.Properties.VariableNames)
        
        % 가장 가까운 조건 찾기
        [~, idx] = min(abs(damping_data.Mach - mach).^2 + ...
                       abs(damping_data.AoA - alpha).^2);
        
        % 감쇠 도함수 (B 프레임) 추출
        if ismember('Clpm', damping_data.Properties.VariableNames)
            damping_coeffs.Clp = damping_data.Clpm(idx);
        else
            damping_coeffs.Clp = -0.2; % 기본값
        end
        
        if ismember('Cmqm', damping_data.Properties.VariableNames)
            damping_coeffs.Cmq = damping_data.Cmqm(idx);
        else
            damping_coeffs.Cmq = -5.0; % 기본값
        end
        
        if ismember('Cyawrm', damping_data.Properties.VariableNames)
            damping_coeffs.Cnr = damping_data.Cyawrm(idx);
        else
            damping_coeffs.Cnr = -5.0; % 기본값
        end
        
        % 감쇠 모멘트 계수가 직접 제공되는 경우
        if ismember('Clmd', damping_data.Properties.VariableNames)
            damping_moments.Clmd = damping_data.Clmd(idx);
        else
            % 계산 (B 프레임이므로 변환 없이 직접 계산)
            damping_moments.Clmd = (omega_b(1) * 0.15/(2*vel)) * damping_coeffs.Clp;
        end
        
        if ismember('Cmmd', damping_data.Properties.VariableNames)
            damping_moments.Cmmd = damping_data.Cmmd(idx);
        else
            % 계산 (B 프레임이므로 변환 없이 직접 계산)
            damping_moments.Cmmd = (omega_b(2) * 0.15/(2*vel)) * damping_coeffs.Cmq;
        end
        
        if ismember('Cyawmd', damping_data.Properties.VariableNames)
            damping_moments.Cnmd = damping_data.Cyawmd(idx);
        else
            % 계산 (B 프레임이므로 변환 없이 직접 계산)
            damping_moments.Cnmd = (omega_b(3) * 0.15/(2*vel)) * damping_coeffs.Cnr;
        end
    else
        % 필요한 열이 없는 경우 기본값 사용
        damping_coeffs.Clp = -0.2;
        damping_coeffs.Cmq = -5.0;
        damping_coeffs.Cnr = -5.0;
        
        % 기본값으로 감쇠 모멘트 계산 (B 프레임)
        damping_moments.Clmd = (omega_b(1) * 0.15/(2*vel)) * damping_coeffs.Clp;
        damping_moments.Cmmd = (omega_b(2) * 0.15/(2*vel)) * damping_coeffs.Cmq;
        damping_moments.Cnmd = (omega_b(3) * 0.15/(2*vel)) * damping_coeffs.Cnr;
    end
else
    % 감쇠 계수 데이터가 없는 경우 기본값 사용
    damping_coeffs.Clp = -0.2;
    damping_coeffs.Cmq = -5.0;
    damping_coeffs.Cnr = -5.0;
    
    % 감쇠 모멘트 계산 (B 프레임)
    damping_moments.Clmd = (omega_b(1) * 0.15/(2*vel)) * damping_coeffs.Clp;
    damping_moments.Cmmd = (omega_b(2) * 0.15/(2*vel)) * damping_coeffs.Cmq;
    damping_moments.Cnmd = (omega_b(3) * 0.15/(2*vel)) * damping_coeffs.Cnr;
end

end