clear all;
close all;
clc;
addpath(genpath('C:\Users\USER\Desktop\hanul_GNC'));
xmlFilePath = 'rocket.xml';
fprintf('\n--- 전체 CG · 관성텐서 계산 시작 ---\n');
% ① XML → 파라미터 트리
rocketParams = extractRocketParams(xmlFilePath);
flatRocketParams = flattenRocketStructure(rocketParams);
% ② 파라미터 → (mass, cg, inertia) 리스트
% compList = buildComponentList(rocketParams);
compList = bclist(flatRocketParams);
% ③ 전체 집계 (평행축 정리 포함)
[MT, CG, ICG] = getAssemblyProps(compList);
fprintf('\n총 질량 : %.3f kg\n', MT);
fprintf('\n전체 CG : [%.4f %.4f %.4f] m (노즈 앞단 기준)\n', CG);
disp('관성 텐서 (전체 CG 기준, kg·m²):');
disp(ICG);


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

