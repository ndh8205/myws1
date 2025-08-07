function flatRocketParams = flattenRocketStructure(rocketParams)
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
    motorInfoList = {}; % 모터 정보를 임시 저장할 셀 배열
    
    for i = 1:numel(rocketParams)
        [processedComps, motorInfo] = processComponent(rocketParams(i), i);
        flatComponents = [flatComponents, processedComps];
        if ~isempty(motorInfo)
            motorInfoList{end+1} = motorInfo;
        end
    end
    
    % ===== ③ 셀 → 구조체 배열 변환 =====
    if isempty(flatComponents)
        flatRocketParams = struct([]);
        return; % 빈 결과 즉시 반환
    else
        flatRocketParams = [flatComponents{:}];
    end
    
    % ===== ④ 모터 정보가 있다면 별도 컴포넌트로 추가 =====
    motorComponents = struct([]);
    for i = 1:numel(motorInfoList)
        if ~isempty(motorInfoList{i})
            % 모터 정보에서 필요한 정보만 추출하여 표준 형식의 컴포넌트 생성
            motorInfo = motorInfoList{i};
            
            % 모터 컴포넌트 생성 (flatRocketParams와 동일한 필드 세트 사용)
            motorComp = createEmptyComponentStruct();
            
            motorComp.Name = [motorInfo.ParentName, '_Motor'];
            motorComp.Type = 'motor';
            motorComp.Finish = 'normal';
            
            % 기하 정보
            if isfield(motorInfo, 'Diameter')
                motorComp.Geometry.Diameter = motorInfo.Diameter;
            end
            if isfield(motorInfo, 'Length')
                motorComp.Geometry.Length = motorInfo.Length;
            end
            
            % 재료 정보
            if isfield(motorInfo, 'Manufacturer')
                motorComp.Material.Name = motorInfo.Manufacturer;
            end
            if isfield(motorInfo, 'Type')
                motorComp.Material.Type = motorInfo.Type;
            end
            
            % 위치 정보
            if isfield(motorInfo, 'AbsoluteStartX')
                motorComp.Position.AbsoluteStartX = motorInfo.AbsoluteStartX;
            end
            if isfield(motorInfo, 'AbsoluteEndX')
                motorComp.Position.AbsoluteEndX = motorInfo.AbsoluteEndX;
            end
            if isfield(motorInfo, 'Length')
                motorComp.Position.Length = motorInfo.Length;
            end
            
            % 모터 특성 정보가 필요하면 추가 필드에 저장
            if isempty(motorComponents)
                motorComponents = motorComp;
            else
                motorComponents(end+1) = motorComp;
            end
        end
    end
    
    % 모터 컴포넌트 배열과 기존 컴포넌트 배열 합치기
    if ~isempty(motorComponents)
        % 먼저 각 배열의 필드가 다른지 확인
        flatFields = fieldnames(flatRocketParams);
        motorFields = fieldnames(motorComponents);
        
        % 필드가 다르면 표준화
        if ~isequal(flatFields, motorFields)
            % 모든 필드의 통합 세트 생성
            allFields = union(flatFields, motorFields);
            
            % flatRocketParams의 모든 구조체에 누락된 필드 추가
            for i = 1:numel(flatRocketParams)
                for j = 1:numel(allFields)
                    if ~isfield(flatRocketParams(i), allFields{j})
                        flatRocketParams(i).(allFields{j}) = [];
                    end
                end
            end
            
            % motorComponents의 모든 구조체에 누락된 필드 추가
            for i = 1:numel(motorComponents)
                for j = 1:numel(allFields)
                    if ~isfield(motorComponents(i), allFields{j})
                        motorComponents(i).(allFields{j}) = [];
                    end
                end
            end
        end
        
        % 이제 안전하게 합칠 수 있음
        flatRocketParams = [flatRocketParams, motorComponents];
    end
    
    % ===== ⑤ 절대 시작 위치(AbsoluteStartX) 기준 오름차순 정렬 =====
    if ~isempty(flatRocketParams) && isfield(flatRocketParams, "Position")
        absX = arrayfun(@(c)defaultOrNaN(c.Position,"AbsoluteStartX"), flatRocketParams);
        [~, idx] = sort(absX);
        flatRocketParams = flatRocketParams(idx);
    end
    
    fprintf('평탄화 완료: 총 %d개 부품 처리됨\n', numel(flatRocketParams));
    
    % ---------------------------------------------------------------------
    % --- 내부 함수들 ---
    % ---------------------------------------------------------------------
    function emptyStruct = createEmptyComponentStruct()
        % 완전히 표준화된 빈 컴포넌트 구조체 생성
        emptyStruct = struct();
        emptyStruct.Name = '';
        emptyStruct.Type = '';
        emptyStruct.Finish = '';
        emptyStruct.Geometry = struct();
        emptyStruct.Material = struct('Name', '', 'Type', '');
        emptyStruct.Position = struct('AbsoluteStartX', NaN, 'AbsoluteEndX', NaN, 'Length', NaN);
        emptyStruct.Mass = NaN;
        emptyStruct.InstanceInfo = struct('Count', 1);
        
        % flatRocketParams에 있는 모든 필드를 여기에 추가
        if ~isempty(flatComponents) && numel(flatComponents) > 0
            sampleComp = flatComponents{1};
            sampleFields = fieldnames(sampleComp);
            for i = 1:numel(sampleFields)
                if ~isfield(emptyStruct, sampleFields{i})
                    emptyStruct.(sampleFields{i}) = [];
                end
            end
        end
    end
    
    function [processedComps, motorInfo] = processComponent(comp, siblingIdx)
        % 결과 초기화
        processedComps = {};
        motorInfo = [];
        
        % 1) 현 부품을 평탄화 목록에 추가
        flatComp = comp;
        
        % 2) 하위 부품 필드 제거 (재귀 전용)
        if isfield(flatComp, "Subcomponents")
            subs = flatComp.Subcomponents;
            flatComp = rmfield(flatComp, "Subcomponents");
        else
            subs = {};
        end
        
        % 3) 모터 정보 추출 (별도로 저장)
        if isfield(flatComp, "MotorMount") && isfield(flatComp.MotorMount, "HasMount") && flatComp.MotorMount.HasMount
            % 모터 정보 구조체 생성
            motorInfo = struct();
            motorInfo.ParentName = flatComp.Name;
            
            if isfield(flatComp.MotorMount, "Motor")
                % 필요한 모터 정보 복사
                if isfield(flatComp.MotorMount.Motor, "Type")
                    motorInfo.Type = flatComp.MotorMount.Motor.Type;
                end
                if isfield(flatComp.MotorMount.Motor, "Manufacturer")
                    motorInfo.Manufacturer = flatComp.MotorMount.Motor.Manufacturer;
                end
                if isfield(flatComp.MotorMount.Motor, "Designation")
                    motorInfo.Designation = flatComp.MotorMount.Motor.Designation;
                end
                if isfield(flatComp.MotorMount.Motor, "Digest")
                    motorInfo.Digest = flatComp.MotorMount.Motor.Digest;
                end
                if isfield(flatComp.MotorMount.Motor, "Diameter")
                    motorInfo.Diameter = flatComp.MotorMount.Motor.Diameter;
                end
                if isfield(flatComp.MotorMount.Motor, "Length")
                    motorInfo.Length = flatComp.MotorMount.Motor.Length;
                end
                if isfield(flatComp.MotorMount.Motor, "Delay")
                    motorInfo.Delay = flatComp.MotorMount.Motor.Delay;
                end
            end
            
            % 위치 정보 계산
            if isfield(flatComp, "Position") && isfield(flatComp.Position, "AbsoluteStartX")
                baseX = flatComp.Position.AbsoluteStartX;
                
                % 마운트 오프셋 추출
                mountOffset = 0;
                if isfield(flatComp.MotorMount, "MountAxialOffset") && ~isnan(flatComp.MotorMount.MountAxialOffset)
                    mountOffset = flatComp.MotorMount.MountAxialOffset;
                elseif isfield(flatComp.MotorMount, "MountPositionValue") && ~isnan(flatComp.MotorMount.MountPositionValue)
                    mountOffset = flatComp.MotorMount.MountPositionValue;
                end
                
                % 모터 돌출 정보 추출
                motorOverhang = 0;
                if isfield(flatComp.MotorMount, "MotorOverhang") && ~isnan(flatComp.MotorMount.MotorOverhang)
                    motorOverhang = flatComp.MotorMount.MotorOverhang;
                end
                
                % 절대 좌표 계산
                motorLength = 0;
                if isfield(motorInfo, "Length")
                    motorLength = motorInfo.Length;
                end
                
                motorInfo.AbsoluteStartX = baseX + mountOffset;
                motorInfo.AbsoluteEndX = motorInfo.AbsoluteStartX + motorLength + motorOverhang;
                motorInfo.Length = motorLength + motorOverhang;
            end
        end
        
        % 4) 필요 없는 필드 제거
        keep = ["Name","Type","Finish","Geometry","Material","Matrial","Position","InstanceInfo","Mass"];
        rm = setdiff(string(fieldnames(flatComp)), keep, 'stable');
        if ~isempty(rm)
            flatComp = rmfield(flatComp, rm);
        end
        
        % 5) 결과 저장
        processedComps{end+1} = flatComp;
        
        % 6) 하위 부품 재귀 처리
        for j = 1:numel(subs)
            [childComps, childMotorInfo] = processComponent(subs{j}, j);
            processedComps = [processedComps, childComps];
            if ~isempty(childMotorInfo)
                motorInfo = childMotorInfo; % 하위에서 모터 정보가 발견되면 저장
            end
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