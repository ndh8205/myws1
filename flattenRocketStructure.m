function flatRocketParams = flattenRocketStructure(rocketParams)
% FLATTENROCKETSTRUCTURE  계층형 로켓 구조체를 평탄화하여 원하는 필드만 남깁니다.
%
%   flatRocketParams = flattenRocketStructure(rocketParams)
%
%   입력
%     rocketParams : extractRocketParams 등으로 얻은 계층형 로켓 파라미터 구조체
%
%   출력
%     flatRocketParams : 하위 부품까지 포함하여 절대좌표 기준으로 평탄화한 구조체 배열
%                        ├─ Name
%                        ├─ Type
%                        ├─ Finish
%                        ├─ Geometry
%                        ├─ Material (또는 Matrial)
%                        ├─ Position
%                        └─ Mass
%
%   나머지 필드는 모두 제거됩니다.
%
% -------------------------------------------------------------------------

    fprintf('\n--- 절대 좌표 기준 로켓 구조 평탄화 시작 ---\n');

    % 평탄화된 부품을 위한 임시 셀 배열
    flatComponents = {};

    % ===== ① 입력 유효성 확인 =====
    if isempty(rocketParams) || ~isstruct(rocketParams)
        fprintf('경고: 입력된 로켓 구조체가 비어있거나 유효하지 않습니다.\n');
        flatRocketParams = struct([]);
        return;
    end

    % ===== ② 각 최상위 부품을 재귀적으로 처리 =====
    for i = 1:numel(rocketParams)
        processComponent(rocketParams(i), i);
    end

    % ===== ③ 셀 → 구조체 배열 변환 =====
    if isempty(flatComponents)
        flatRocketParams = struct([]);
    else
        flatRocketParams = [flatComponents{:}];
    end

    % ===== ④ 절대 시작 위치(AbsoluteStartX) 기준 오름차순 정렬 =====
    if ~isempty(flatRocketParams) && isfield(flatRocketParams, "Position")
        absX = arrayfun(@(c)defaultOrNaN(c.Position,"AbsoluteStartX"), flatRocketParams);
        [~, idx] = sort(absX);
        flatRocketParams = flatRocketParams(idx);
    end

    fprintf('평탄화 완료: 총 %d개 부품 처리됨\n', numel(flatRocketParams));

    % ---------------------------------------------------------------------
    %                         --- 내부 함수들 ---
    % ---------------------------------------------------------------------
    function processComponent(comp, siblingIdx)
        % 1) 현 부품을 평탄화 목록에 추가
        flatComp = comp;

        % 2) 하위 부품 필드 제거 (재귀 전용)
        if isfield(flatComp, "Subcomponents")
            subs = flatComp.Subcomponents;
            flatComp = rmfield(flatComp, "Subcomponents");
        else
            subs = {};
        end

        % 3) 필요 없는 필드 제거 ―――――――――――――――――――――――――――――――――――
        keep = ["Name","Type","Finish","Geometry","Material","Matrial","Position","InstanceInfo","Mass"];
        rm    = setdiff(string(fieldnames(flatComp)), keep, 'stable');
        if ~isempty(rm)
            flatComp = rmfield(flatComp, rm);
        end

        % 4) 결과 저장
        flatComponents{end+1} = flatComp;

        % 5) 하위 부품 재귀 처리
        for j = 1:numel(subs)
            processComponent(subs{j}, j);
        end
    end

    % Missing-field → NaN 헬퍼
    function val = defaultOrNaN(s, f)
        if isfield(s, f) && ~isempty(s.(f))
            val = s.(f);
        else
            val = NaN;
        end
    end
end
