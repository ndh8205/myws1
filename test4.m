clear all;
close all;
clc;
addpath(genpath('C:\Users\DDHD\Desktop\hanul_GNC'));
xmlFilePath = 'rocket.xml';
fprintf('\n--- 전체 CG · 관성텐서 계산 시작 ---\n');
% ① XML → 파라미터 트리
rocketParams = extractRocketParams(xmlFilePath);
flatRocketParams = flattenRocketStructure(rocketParams);
% ② 파라미터 → (mass, cg, inertia) 리스트
compList = bclist(flatRocketParams);
% ③ 전체 집계 (평행축 정리 포함)
[MT, CG, ICG] = getAssemblyProps(compList);
fprintf('\n총 질량 : %.3f kg\n', MT);
fprintf('\n전체 CG : [%.4f %.4f %.4f] m (노즈 앞단 기준)\n', CG);
disp('관성 텐서 (전체 CG 기준, kg·m²):');
disp(ICG);

% -- 여기서부터 핀셋 시각화 추가 --
% 핀셋 이름 지정 (여기서는 예시로 'Airfoil_Fins' 사용)
finSetName = 'Airfoil_Fins';

% 해당 핀셋이 존재하는지 확인
hasFinSet = false;
for i = 1:numel(compList)
    if isfield(compList(i), 'name') && contains(compList(i).name, finSetName)
        hasFinSet = true;
        break;
    end
end

% 핀셋이 존재하면 시각화 실행
if hasFinSet
    fprintf('\n핀셋 "%s" 시각화 실행\n', finSetName);
    visualizeFinSet(compList, finSetName);
else
    fprintf('\n핀셋 "%s"을(를) 찾을 수 없습니다.\n', finSetName);
    
    % 대신 사용 가능한 핀셋 이름 출력
    fprintf('사용 가능한 핀셋:\n');
    finNames = {};
    for i = 1:numel(compList)
        if isfield(compList(i), 'name') && strcmpi(compList(i).type, 'fin')
            % 핀 이름에서 숫자 제거하여 기본 이름만 추출
            baseName = regexprep(compList(i).name, '_\d+$', '');
            if ~ismember(baseName, finNames)
                finNames{end+1} = baseName;
                fprintf('  - %s\n', baseName);
            end
        end
    end
    
    % 사용자에게 입력 요청
    if ~isempty(finNames)
        fprintf('\n시각화할 핀셋 이름을 입력하세요 (위 목록에서 선택): ');
        userInput = input('', 's');
        if ~isempty(userInput)
            visualizeFinSet(compList, userInput);
        end
    end
end

% -- 여기까지 핀셋 시각화 추가 --

N = numel(compList);
names = cell(N,1); % 이름(문자열)
mass = zeros(N,1); % 질량
cgx = zeros(N,1); % CG x-좌표
for k = 1:N
    % name 필드가 비어 있으면 타입_인덱스로 대체
    if isfield(compList(k),'name') && ~isempty(compList(k).name)
        names{k} = compList(k).name;
    else
        names{k} = [compList(k).Type '_' num2str(k)];
    end
    % mass·CGx 가 비어있어도 0 / NaN 대신 0 으로
    m_k = compList(k).mass;
    if isempty(m_k) || ~isscalar(m_k) || isnan(m_k), m_k = 0; end
    mass(k) = m_k;
    cg = compList(k).cg_ref;
    if numel(cg)~=3 || any(isnan(cg)), cgx(k)=0;
    else, cgx(k)=cg(1);
    end
end

tbl = table(names,mass,cgx,'VariableNames',{'Name','Mass','CGx'});
% (Name + CGx) 문자열을 키로 만들어 중복 횟수 계산
keyStr = strcat(tbl.Name,"__",compose("%.3f",tbl.CGx));
[keyUni,~,idx] = unique(keyStr); % idx : 각 행이 keyUni 몇 번째?
cnt = accumarray(idx,1); % key별 개수
dupRows = cnt(idx) > 1; % 두 번 이상인 행
dupTbl = tbl(dupRows,:);
fprintf('\n 두 번 이상 집계된 항목 (Name | Mass[kg] | CGx[m])\n');

if isempty(dupTbl)
    disp(' → 중복 집계된 항목이 없습니다.');
else
    disp(dupTbl)
end

N = numel(compList);
names = cell(N,1); mass = zeros(N,1); cgx = zeros(N,1);

for k = 1:N
    % 이름이 비었으면 타입+인덱스로 대체
    nm = ''; if isfield(compList(k),'name'), nm = compList(k).name; end
    if isempty(nm), nm = [compList(k).Type '_' num2str(k)]; end
    names{k} = nm;
    % 질량·CG-x 안전 추출
    m_k = compList(k).mass;
    if isempty(m_k) || ~isscalar(m_k) || isnan(m_k), m_k = 0; end
    mass(k) = m_k;
    cg = compList(k).cg_ref;
    if numel(cg)==3 && all(isfinite(cg)), cgx(k) = cg(1); end
end

rankTbl = table(names,mass,cgx,'VariableNames',{'Name','Mass_kg','CGx_m'});
rankTbl = sortrows(rankTbl,'Mass_kg','descend');
disp('질량이 큰 순서 TOP 20');
disp(rankTbl(1:min(100,height(rankTbl)),:));