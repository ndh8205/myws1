function aero_data = load_aero_data(force_moment_path, damping_path)
% load_aero_data  공력계수 데이터를 두 CSV 파일에서 읽어오는 함수
%
% aero_data = load_aero_data(force_moment_path, damping_path)
%
% 입력:
% force_moment_path : 정적 공력계수 데이터가 포함된 CSV 파일 경로
% damping_path : 감쇠 계수 데이터가 포함된 CSV 파일 경로
%
% 출력:
% aero_data : 공력계수 데이터를 담은 구조체

% 파일 경로 확인
if ~exist(force_moment_path, 'file')
    error('load_aero_data:FileNotFound', '정적 계수 파일을 찾을 수 없습니다: %s', force_moment_path);
end

% 1. 정적 공력계수 로드 (Force & Moment)
fprintf('정적 공력계수 데이터 로드 중: %s\n', force_moment_path);
try
    opts = detectImportOptions(force_moment_path, 'VariableNamingRule', 'preserve');
    static_data = readtable(force_moment_path, opts);
    
    % 열 이름 확인
    required_cols = {'Velocity', 'Mach', 'AoA', 'CA', 'CN', 'CY', 'Clm', 'Cmm', 'Cyawm'};
    missing_cols = setdiff(required_cols, static_data.Properties.VariableNames);
    
    if ~isempty(missing_cols)
        error('load_aero_data:MissingColumns', '정적 계수 파일에 필요한 열이 누락되었습니다: %s', ...
            strjoin(missing_cols, ', '));
    end
    
    % 열 이름을 내부 표준으로 변환 (B 프레임 기준)
    static_data.Properties.VariableNames{'CA'} = 'CA';
    static_data.Properties.VariableNames{'CN'} = 'CN';
    static_data.Properties.VariableNames{'CY'} = 'CY';
    static_data.Properties.VariableNames{'Clm'} = 'Cl';
    static_data.Properties.VariableNames{'Cmm'} = 'Cm';
    static_data.Properties.VariableNames{'Cyawm'} = 'Cn';
    
    % 고유한 받음각과 마하수 배열 추출
    alpha_arr = unique(static_data.AoA);
    mach_arr = unique(static_data.Mach);
    vel_arr = unique(static_data.Velocity);
    
    fprintf('  정적 계수 데이터 로드 완료: %d개 받음각, %d개 마하수, %d개 속도 지점\n', ...
        length(alpha_arr), length(mach_arr), length(vel_arr));
    
    % 정적 계수 데이터를 구조체로 구성
    static_coeffs = struct();
    static_coeffs.data = static_data;
    static_coeffs.alpha_arr = alpha_arr;
    static_coeffs.mach_arr = mach_arr;
    static_coeffs.velocity_arr = vel_arr;
    
catch ME
    error('load_aero_data:ReadError', '정적 계수 파일 읽기 오류: %s', ME.message);
end

% 2. 감쇠 계수 로드 (Damping coefficients)
damping_derivs = struct();
if exist(damping_path, 'file')
    fprintf('감쇠 계수 데이터 로드 중: %s\n', damping_path);
    try
        opts = detectImportOptions(damping_path, 'VariableNamingRule', 'preserve');
        damping_data = readtable(damping_path, opts);
        
        % B 프레임 기준이므로 열 이름 그대로 유지
        % 열 이름 확인
        required_cols = {'Mach', 'velocity', 'AoA', 'deg', 'rad/s', 'Clpm', 'Cmqm', 'Cyawrm', 'Clmd', 'Cmmd', 'Cyawmd'};
        existing_cols = intersect(required_cols, damping_data.Properties.VariableNames);
        
        if length(existing_cols) < 5  % 최소한 5개 열은 있어야 함
            warning('load_aero_data:MissingColumns', '감쇠 계수 파일에 중요 열이 부족합니다.');
        end
        
        % 감쇠 계수 데이터 구조체 구성
        damping_derivs.data = damping_data;
        
        % 공통 파라미터 추출 (있는 경우)
        if ismember('AoA', damping_data.Properties.VariableNames)
            damping_derivs.alpha_arr = unique(damping_data.AoA);
        end
        if ismember('Mach', damping_data.Properties.VariableNames)
            damping_derivs.mach_arr = unique(damping_data.Mach);
        end
        if ismember('velocity', damping_data.Properties.VariableNames)
            damping_derivs.velocity_arr = unique(damping_data.velocity);
        end
        
        fprintf('  감쇠 계수 데이터 로드 완료\n');
    catch ME
        warning('load_aero_data:DampingReadError', '감쇠 계수 파일 로드 실패: %s\n기본값으로 대체합니다.', ME.message);
        damping_derivs.data = [];
    end
else
    warning('load_aero_data:DampingFileNotFound', '감쇠 계수 파일을 찾을 수 없습니다. 기본값으로 계속합니다.');
    damping_derivs.data = [];
end

% 통합 데이터 구조체 생성
aero_data = struct();
aero_data.static_coeffs = static_coeffs;
aero_data.damping_derivs = damping_derivs;
aero_data.alpha_arr = alpha_arr;
aero_data.mach_arr = mach_arr;
aero_data.velocity_arr = vel_arr;
aero_data.frame = 'B';  % B 프레임 데이터임을 명시

fprintf('공력 데이터 로드 완료\n');
end