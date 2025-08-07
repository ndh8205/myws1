function [MT, CG, ICG, components] = vehicle_mass_calc(flatRocketParams, t, motorCsvPath, staticCache)
% vehicle_mass_calc  로켓 질량 특성(질량, CG, 관성 텐서) 계산 함수
%
% [MT, CG, ICG, components] = vehicle_mass_calcW(flatRocketParams, t, motorCsvPath, staticCache)
%
% 입력:
% flatRocketParams : 평탄화된 로켓 파라미터 구조체 (flattenRocketStructure 함수 결과)
% t : 현재 시뮬레이션 시간 [s]
% motorCsvPath : 모터 추력/질량 데이터 CSV 파일 경로
% staticCache : (선택) 정적 부품 질량 특성 캐시 (반복 계산 방지용)
%
% 출력:
% MT : 총 질량 [kg]
% CG : 무게중심 3D 좌표 [x, y, z] [m]
% ICG : 무게중심 기준 관성 텐서 (3x3) [kg-m^2]
% components : 계산에 사용된 부품 질량 특성 목록 {정적부품, 모터}

% persistent 변수를 사용하여 정적 부품 계산 캐싱 (성능 향상)
persistent static_components last_params_hash

% 입력 검증
if nargin < 3
    error('vehicle_mass_calc:입력부족', '최소 3개 입력이 필요합니다: flatRocketParams, t, motorCsvPath');
end

% 선택적 입력: 정적 부품 캐시
if nargin >= 4 && ~isempty(staticCache)
    static_components = staticCache;
else
    % 1. 해시 값 계산 (flatRocketParams가 변경되었는지 확인)
    try
        % 구조체를 문자열로 변환하여 간단한 해시 생성
        params_str = jsonencode(flatRocketParams);
        current_hash = sum(params_str);
    catch
        % JSON 변환 실패 시 항상 재계산
        current_hash = rand();
    end
    
    % 2. 정적 부품 질량 특성 계산 (캐싱 사용)
    if isempty(static_components) || isempty(last_params_hash) || current_hash ~= last_params_hash
        static_components = static_param(flatRocketParams);
        last_params_hash = current_hash;
        fprintf('정적 부품 질량 특성 계산 완료 (캐시 생성)\n');
    else
        fprintf('정적 부품 질량 특성 캐시 사용\n');
    end
end

% 3. 모터 상태 계산 (현재 시간 기준)
motor_component = motor_state(flatRocketParams, t, motorCsvPath);

% 4. 전체 질량 특성 계산 (정적 부품 + 모터)
combined_components = {static_components, motor_component};
[MT, CG, ICG] = getAssemblyProps(combined_components);

% 5. 결과 컴포넌트 목록 반환 (선택적)
if nargout >= 4
    components = combined_components;
end

% 결과 오류 검사
if any(isnan(MT)) || any(isnan(CG)) || any(isnan(ICG(:)))
    warning('vehicle_mass_calc:계산오류', '질량 특성 계산 중 NaN 값 발생');
end

% % 디버그 출력 (주석 처리)
% fprintf('T=%.2f초 → M=%.3fkg, CG=[%.3f, %.3f, %.3f]m\n', t, MT, CG(1), CG(2), CG(3));
end