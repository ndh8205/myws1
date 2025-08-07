% MATLAB Script to Augment Aerodynamic Data using Extrapolation

clear all; 
close all;
clc;
addpath(genpath('C:\Users\USER\Desktop\hanul_GNC'));

% --- 1. Load Data ---
fileName = 'csv_Force_moment.csv';
try
    T = readtable(fileName);
    disp('데이터 로딩 완료.');
catch ME
    if strcmp(ME.identifier, 'MATLAB:readtable:FileNotFound')
        error('오류: ''%s'' 파일을 찾을 수 없습니다. 스크립트와 동일한 경로에 있는지 확인하세요.', fileName);
    else
        rethrow(ME);
    end
end

% --- 기본 정보 확인 (선택 사항) ---
disp('데이터 테이블 정보:');
summary(T);
disp('데이터 샘플 (처음 5행):');
disp(head(T, 5));

% --- 컬럼 이름 정의 ---
machCol = 'Mach';
aoaCol = 'AoA';
coeffCols = {'CA', 'CN', 'CY', 'Clm', 'Cmm', 'Cyawm'};
velocityCol = 'Velocity'; % Velocity 컬럼도 사용

% 필수 컬럼 존재 확인
requiredCols = [machCol, aoaCol, velocityCol, coeffCols];
missingCols = setdiff(requiredCols, T.Properties.VariableNames);
if ~isempty(missingCols)
    error('오류: 다음 필수 컬럼이 없습니다: %s', strjoin(missingCols, ', '));
end

% --- 2. Calculate Slopes at Mach = 0.83 ---
machForSlope = 0.83;
% 부동 소수점 비교를 위한 허용 오차 사용
tolerance = 1e-6;
T_m083 = T(abs(T.(machCol) - machForSlope) < tolerance, :);
T_m083 = sortrows(T_m083, aoaCol); % AoA 기준으로 정렬

if height(T_m083) < 2
    error('오류: 마하수 %.2f에서 기울기를 계산하기에 데이터가 부족합니다 (최소 2개 필요).', machForSlope);
end

slopes_deg = containers.Map('KeyType', 'char', 'ValueType', 'double'); % 기울기 저장 (단위: /degree)

fprintf('\n--- 마하수 %.2f에서 받음각(AoA) 대비 계수 기울기 계산 (단위: /degree) ---\n', machForSlope);
for i = 1:length(coeffCols)
    col = coeffCols{i};
    x_data = T_m083.(aoaCol);
    y_data = T_m083.(col);

    % 데이터 포인트가 2개 이상일 때만 polyfit 사용
    if length(unique(x_data)) > 1
        % polyfit(x, y, 1)은 [slope, intercept]를 반환
        p = polyfit(x_data, y_data, 1);
        slope = p(1);
        slopes_deg(col) = slope;

        % R^2 계산 (선택 사항)
        y_fit = polyval(p, x_data);
        y_resid = y_data - y_fit;
        SS_resid = sum(y_resid.^2);
        SS_total = (length(y_data)-1) * var(y_data);
        rsq = 1 - SS_resid/SS_total;
        fprintf("'%s' 기울기: %.6f /deg (R^2: %.4f)\n", col, slope, rsq);
    else
        slopes_deg(col) = 0.0; % 단일 데이터 포인트면 기울기 0으로 가정
        fprintf("'%s' 기울기: 0.0 (단일 AoA 데이터 포인트)\n", col);
    end
end

% --- 3. Identify Target Machs and AoAs for Extrapolation ---
unique_machs = unique(T.(machCol));
target_machs = unique_machs(unique_machs > machForSlope + tolerance); % 0.83 초과 마하수
target_aoas = unique(T_m083.(aoaCol));
target_aoas = target_aoas(abs(target_aoas - 0) > tolerance); % 0이 아닌 AoA 값들

if isempty(target_machs)
    disp('외삽할 마하수 > 0.83 대상이 없습니다.');
    return; % 추가 작업 없이 종료
end
if isempty(target_aoas)
    disp('외삽할 받음각 > 0 대상이 없습니다 (M=0.83 데이터 확인 필요).');
    return; % 추가 작업 없이 종료
end

fprintf('\n외삽 대상 마하수: %s\n', num2str(target_machs'));
fprintf('외삽 대상 받음각 (0 제외): %s\n', num2str(target_aoas'));

% --- 4. Perform Extrapolation and Generate New Data ---
newRows = cell(length(target_machs) * length(target_aoas), width(T)); % 새로운 데이터를 저장할 셀 배열 초기화
rowIndex = 1;
disp('외삽 진행 중...');

for i = 1:length(target_machs)
    m = target_machs(i);
    % 현재 마하수(m)에서 AoA=0 인 베이스 행 찾기
    base_row_idx = find(abs(T.(machCol) - m) < tolerance & abs(T.(aoaCol) - 0) < tolerance);

    if isempty(base_row_idx)
        warning('마하수 %.2f, 받음각 0도에 대한 베이스 데이터를 찾을 수 없습니다. 이 마하수는 건너뜁니다.', m);
        continue;
    end
    % 중복된 베이스 행이 있을 경우 첫 번째 사용
    base_row = T(base_row_idx(1), :);

    for j = 1:length(target_aoas)
        alpha = target_aoas(j);

        % 새 행 데이터 생성
        new_row_data = table(); % 임시 테이블 사용 가능 또는 직접 셀 배열 채우기
        new_row_data.(machCol) = m;
        new_row_data.(aoaCol) = alpha;
        new_row_data.(velocityCol) = base_row.(velocityCol); % 베이스 행의 Velocity 값 사용

        % 각 계수 외삽
        for k = 1:length(coeffCols)
            col = coeffCols{k};
            base_value = base_row.(col);
            slope = slopes_deg(col);
            extrapolated_value = base_value + slope * alpha; % alpha는 degree 단위
            new_row_data.(col) = extrapolated_value;
        end
        % 셀 배열에 행 데이터 추가 (테이블 변수 순서에 맞춰야 함)
        newRows(rowIndex, :) = table2cell(new_row_data(1, T.Properties.VariableNames)); % 원본 테이블 순서대로 저장
        rowIndex = rowIndex + 1;
    end
end

% 사용되지 않은 셀 배열 공간 제거
newRows(rowIndex:end, :) = [];

disp('외삽 완료.');

% --- 5. Combine Original and Extrapolated Data ---
if ~isempty(newRows)
    T_new = cell2table(newRows, 'VariableNames', T.Properties.VariableNames);

    % 데이터 타입 맞추기 (필요시) - cell2table이 자동으로 추론하지만, 가끔 필요할 수 있음
    for colName = T.Properties.VariableNames
        if isnumeric(T.(colName{1})(1)) && ~isnumeric(T_new.(colName{1})(1))
            T_new.(colName{1}) = str2double(T_new.(colName{1})); % 예시: 문자를 숫자로
        end
         % 필요한 다른 타입 변환 추가
    end

    T_augmented = [T; T_new]; % 테이블 수직 결합

    % 최종 테이블 정렬
    T_augmented = sortrows(T_augmented, {machCol, aoaCol});

    disp('--- 보강된 데이터 테이블 (일부 확인) ---');
    disp('데이터 시작 부분:');
    disp(head(T_augmented, 5));

    disp('데이터 중간 부분 (M=0.83 근처):');
    disp(T_augmented(T_augmented.(machCol) >= 0.8 & T_augmented.(machCol) <= 0.86, :));

    disp('데이터 끝 부분:');
    disp(tail(T_augmented, 5));

    % 보강 후 AoA 분포 확인 (예: M=1.0)
    disp('보강 후 마하수 1.0 에서의 받음각 분포:');
    disp(unique(T_augmented.(aoaCol)(abs(T_augmented.(machCol) - 1.0) < tolerance))');

    % --- (선택 사항) 보강된 데이터 저장 ---
    outputFileName = 'csv_Force_moment_arg.csv';
    writetable(T_augmented, outputFileName);
    fprintf('\n보강된 데이터가 ''%s''으로 저장되었습니다.\n', outputFileName);

else
    disp('새로운 데이터가 생성되지 않아 원본 데이터가 변경되지 않았습니다.');
    T_augmented = T; % 변경 없음
end

% 이제 T_augmented 테이블을 사용하여 MATLAB에서 보간 등의 후속 작업을 진행할 수 있습니다.