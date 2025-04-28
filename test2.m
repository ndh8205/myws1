clear all
close all
clc

addpath(genpath('C:\Users\USER\Desktop\hanul_GNC'));


xmlFilePath = 'rocket.xml';



fprintf('--- 로켓 데이터 추출 시작 ---\n');
hsr_1 = extractRocketParams(xmlFilePath);

fprintf('\n--- 물리량 계산 및 집계 시작 (노즈콘 끝 기준) ---\n');


allComponentProps = {};


if ~isempty(hsr_1)
    allComponentProps = calculateAndCollectProperties(hsr_1, allComponentProps);
end

if ~isempty(allComponentProps)
    processedPropsStructArray = [allComponentProps{:}];
else
    processedPropsStructArray = struct('mass', {}, 'cg_ref', {}, 'inertia_cg', {}); % 빈 구조체 배열 생성
end


[totalMass, overallCG_tip, totalInertiaTensor_overallCG] = getAssemblyProps(processedPropsStructArray);

fprintf('\n--- 계산 결과 ---\n');
fprintf('총 질량: %.4f kg\n', totalMass);
fprintf('노즈콘 끝 기준 전체 CG 위치 (X, Y, Z): [%.4f, %.4f, %.4f] m\n', overallCG_tip(1), overallCG_tip(2), overallCG_tip(3));
fprintf('전체 CG 기준 관성 모멘트 텐서 (일부 근사 포함):\n');
disp(totalInertiaTensor_overallCG);