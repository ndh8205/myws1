function visualizeFinSet(compList, finSetName)
% visualizeFinSet  핀셋의 배치와 특성을 시각화합니다.
% 절대 좌표로 평탄화된 로켓 구조체에 맞게 개선된 버전
%
% visualizeFinSet(compList, finSetName)
%
% 입력
% compList : bclist 함수로 얻은 부품 리스트
% finSetName : 시각화할 핀셋의 이름 (예: 'Airfoil_Fins')

% 핀 관련 부품 찾기
finIndices = [];
allBodyIndices = [];

% 핀셋 및 본체 부품 인덱스 찾기
for i = 1:numel(compList)
    % 이름과 타입으로 핀 및 본체 찾기
    if isfield(compList(i), 'name') && isfield(compList(i), 'type')
        % 핀 찾기
        if strcmpi(compList(i).type, 'fin') && contains(compList(i).name, finSetName)
            finIndices(end+1) = i;
        end
        
        % 본체 찾기 (시각화용)
        if any(strcmpi(compList(i).type, {'bodytube', 'nosecone', 'transition'}))
            allBodyIndices(end+1) = i;
        end
    end
end

if isempty(finIndices)
    error('핀셋 "%s"을(를) 찾을 수 없습니다.', finSetName);
end

% 핀 데이터 추출
finMasses = zeros(length(finIndices), 1);
finCGs = zeros(length(finIndices), 3);
finInertias = zeros(3, 3, length(finIndices));

for i = 1:length(finIndices)
    idx = finIndices(i);
    finMasses(i) = compList(idx).mass;
    finCGs(i, :) = compList(idx).cg_ref;
    finInertias(:, :, i) = compList(idx).inertia_cg;
end

% 핀 위치 정보 출력 (디버깅용)
fprintf('\n핀셋 "%s" 데이터:\n', finSetName);
fprintf('-------------------------------------------\n');
fprintf('총 %d개 핀 발견\n', length(finIndices));

% 핀 각도 계산 (YZ 평면 각도)
finAngles = atan2(finCGs(:, 3), finCGs(:, 2));
meanX = mean(finCGs(:, 1));  % X 위치 평균

% 전체 핀셋의 질량 중심 계산
totalMass = sum(finMasses);
finSetCG = sum(finMasses .* finCGs) / totalMass;

% 핀 배열 각도 및 위치 요약
fprintf('핀 배열 정보:\n');
fprintf('  X 위치: %.4f m\n', meanX);
fprintf('  평균 반경: %.4f m\n', mean(sqrt(finCGs(:,2).^2 + finCGs(:,3).^2)));
for i = 1:length(finIndices)
    fprintf('  핀 %d: 각도=%.1f°, 반경=%.4f m, 위치=[%.4f, %.4f, %.4f]\n', ...
            i, rad2deg(finAngles(i)), ...
            sqrt(finCGs(i,2)^2 + finCGs(i,3)^2), ...
            finCGs(i,1), finCGs(i,2), finCGs(i,3));
end

% 핀셋 CG 및 관성 정보
fprintf('핀셋 질량 특성:\n');
fprintf('  총 질량: %.4f kg\n', totalMass);
fprintf('  CG 위치: [%.4f, %.4f, %.4f] m\n', finSetCG(1), finSetCG(2), finSetCG(3));
fprintf('  관성 텐서 대각 요소: [%.6f, %.6f, %.6f] kg·m²\n', ...
        finInertias(1,1,1), finInertias(2,2,1), finInertias(3,3,1));

% 3D 시각화 창 생성
figure('Name', ['핀셋 시각화: ' finSetName], 'NumberTitle', 'off', 'Color', 'white');
set(gcf, 'Position', [100, 100, 800, 600]);

% 3D 축 생성
ax = axes;
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
view(ax, 3);
xlabel('X (Axial, m)');
ylabel('Y (m)');
zlabel('Z (m)');
title(['핀셋 시각화: ' finSetName]);

% 로켓 범위 계산 (X 범위, 반경)
if ~isempty(allBodyIndices)
    % 본체 부품 기반 범위 계산
    bodyCGs = zeros(length(allBodyIndices), 3);
    for i = 1:length(allBodyIndices)
        bodyCGs(i,:) = compList(allBodyIndices(i)).cg_ref;
    end
    
    % 로켓 X 범위 및 대략적인 반경 추정
    rocketXMin = min(bodyCGs(:,1)) - 0.2;
    rocketXMax = max(bodyCGs(:,1)) + 0.2;
    approxRadius = 0.1;  % 기본값
else
    % 본체 부품 없을 경우 핀 위치 기반 범위 계산
    rocketXMin = min(finCGs(:,1)) - 0.2;
    rocketXMax = max(finCGs(:,1)) + 0.2;
    approxRadius = mean(sqrt(finCGs(:,2).^2 + finCGs(:,3).^2)) * 0.7;
end

% 핀이 부착된 로켓 본체 위치 추정
finX = mean(finCGs(:,1));  % 핀 X 위치 평균

% 3D 좌표축 그리기
axisLength = max([rocketXMax - rocketXMin, 0.3]);
quiver3(0, 0, 0, axisLength, 0, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.1);
quiver3(0, 0, 0, 0, axisLength/2, 0, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.1);
quiver3(0, 0, 0, 0, 0, axisLength/2, 0, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.1);
text(axisLength*1.05, 0, 0, 'X', 'Color', 'r', 'FontWeight', 'bold');
text(0, axisLength/2*1.05, 0, 'Y', 'Color', 'g', 'FontWeight', 'bold');
text(0, 0, axisLength/2*1.05, 'Z', 'Color', 'b', 'FontWeight', 'bold');

% 로켓 본체 그리기 (간략하게)
bodyRadius = mean(sqrt(finCGs(:,2).^2 + finCGs(:,3).^2)) * 0.7;  % 핀 부착 반경의 약 70%로 추정
[X, Y, Z] = cylinder(bodyRadius, 30);
X = rocketXMin + (rocketXMax - rocketXMin) * X;  % X 범위 조정
h_body = surf(X, Y, Z, 'FaceColor', [0.7 0.7 0.7], 'FaceAlpha', 0.3, 'EdgeAlpha', 0.1);

% 핀 그리기 - 사다리꼴 형상 추정
for i = 1:length(finIndices)
    % 현재 핀의 데이터
    cg = finCGs(i,:);
    angle = finAngles(i);
    
    % 핀 형상 추정 (CG 위치 기반)
    finHeight = sqrt(cg(2)^2 + cg(3)^2) - bodyRadius;  % 핀 높이 추정
    rootChord = 0.15;  % 루트 코드 추정
    tipChord = 0.08;   % 팁 코드 추정
    sweepLength = 0.05; % 스윕 추정
    thickness = 0.005;  % 두께 추정
    
    % 핀 X 위치 (CG 기준)
    finFront = cg(1) - rootChord/2;
    
    % 사다리꼴 핀 꼭지점 계산
    verts = [
        finFront, bodyRadius*cos(angle), bodyRadius*sin(angle);                   % 루트 앞쪽
        finFront+rootChord, bodyRadius*cos(angle), bodyRadius*sin(angle);         % 루트 뒤쪽
        finFront+rootChord-sweepLength+tipChord, (bodyRadius+finHeight)*cos(angle), (bodyRadius+finHeight)*sin(angle); % 팁 뒤쪽
        finFront+sweepLength, (bodyRadius+finHeight)*cos(angle), (bodyRadius+finHeight)*sin(angle)  % 팁 앞쪽
    ];
    
    % 핀 면 그리기 (상단)
    h_fin = patch('Vertices', verts, 'Faces', [1,2,3,4], 'FaceColor', [0.7 0.2 0.2], 'EdgeColor', 'k', 'FaceAlpha', 0.8);
    
    % 핀 CG 표시
    h_cg = scatter3(cg(1), cg(2), cg(3), 50, 'ro', 'filled');
end

% 핀셋 CG 표시
h_setcg = scatter3(finSetCG(1), finSetCG(2), finSetCG(3), 100, 'go', 'filled');

% 관성 주축 계산 및 표시
[eigVecs, eigVals] = eig(finInertias(:,:,1));  % 첫 번째 핀의 관성텐서 사용
[~, idx] = sort(diag(eigVals));
eigVecs = eigVecs(:, idx);
eigVals = diag(eigVals);
eigVals = eigVals(idx);

% 관성 주축 표시 (핀셋 CG 위치에서)
axisScale = finHeight/2;
quiver3(finSetCG(1), finSetCG(2), finSetCG(3), eigVecs(1,1)*axisScale, eigVecs(2,1)*axisScale, eigVecs(3,1)*axisScale, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.3);
quiver3(finSetCG(1), finSetCG(2), finSetCG(3), eigVecs(1,2)*axisScale, eigVecs(2,2)*axisScale, eigVecs(3,2)*axisScale, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.3);
quiver3(finSetCG(1), finSetCG(2), finSetCG(3), eigVecs(1,3)*axisScale, eigVecs(2,3)*axisScale, eigVecs(3,3)*axisScale, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.3);

% 범례 추가
legend([h_body, h_fin, h_cg, h_setcg], {'로켓 본체', '핀', '핀 CG', '핀셋 CG'}, 'Location', 'best');

% 뷰 조정
view(30, 20);

% 평면도 창 생성 (별도 창)
figure('Name', ['핀셋 평면도: ' finSetName], 'NumberTitle', 'off', 'Color', 'white');
set(gcf, 'Position', [100, 700, 1000, 400]);

% XY 평면도 (위에서 본 모습)
subplot(1,3,1);
hold on;
grid on;
title('XY 평면도 (위에서 본 모습)');
xlabel('X (m)');
ylabel('Y (m)');
axis equal;

% 로켓 본체 그리기
plot([rocketXMin, rocketXMax], [0, 0], 'k-', 'LineWidth', 2);  % 중심축
rectangle('Position', [rocketXMin, -bodyRadius, rocketXMax-rocketXMin, 2*bodyRadius], 'Curvature', [1,1], 'EdgeColor', [0.5 0.5 0.5], 'LineWidth', 1.5);

% 핀 CG 그리기
scatter(finCGs(:,1), finCGs(:,2), 50, 'ro', 'filled');

% 핀셋 CG 그리기
scatter(finSetCG(1), finSetCG(2), 100, 'go', 'filled');

% 핀 윤곽 그리기
for i = 1:length(finIndices)
    cg = finCGs(i,:);
    angle = finAngles(i);
    
    % 핀 형상 추정 (XY 평면에서)
    finHeight = sqrt(cg(2)^2 + cg(3)^2) - bodyRadius;
    rootChord = 0.15;
    tipChord = 0.08;
    sweepLength = 0.05;
    
    % 핀 X 위치
    finFront = cg(1) - rootChord/2;
    
    % 핀 윤곽선 (XY 평면 투영)
    x = [finFront, finFront+rootChord, finFront+rootChord-sweepLength+tipChord, finFront+sweepLength, finFront];
    y = [bodyRadius*cos(angle), bodyRadius*cos(angle), (bodyRadius+finHeight)*cos(angle), (bodyRadius+finHeight)*cos(angle), bodyRadius*cos(angle)];
    
    % 핀 그리기
    patch(x, y, [0.7 0.2 0.2], 'EdgeColor', 'k');
end

% XZ 평면도 (옆에서 본 모습)
subplot(1,3,2);
hold on;
grid on;
title('XZ 평면도 (옆에서 본 모습)');
xlabel('X (m)');
zlabel('Z (m)');
axis equal;

% 로켓 본체 그리기
plot([rocketXMin, rocketXMax], [0, 0], 'k-', 'LineWidth', 2);  % 중심축
rectangle('Position', [rocketXMin, -bodyRadius, rocketXMax-rocketXMin, 2*bodyRadius], 'Curvature', [1,1], 'EdgeColor', [0.5 0.5 0.5], 'LineWidth', 1.5);

% 핀 CG 그리기
scatter(finCGs(:,1), finCGs(:,3), 50, 'ro', 'filled');

% 핀셋 CG 그리기
scatter(finSetCG(1), finSetCG(3), 100, 'go', 'filled');

% 핀 윤곽 그리기
for i = 1:length(finIndices)
    cg = finCGs(i,:);
    angle = finAngles(i);
    
    % 핀 형상 추정 (XZ 평면에서)
    finHeight = sqrt(cg(2)^2 + cg(3)^2) - bodyRadius;
    rootChord = 0.15;
    tipChord = 0.08;
    sweepLength = 0.05;
    
    % 핀 X 위치
    finFront = cg(1) - rootChord/2;
    
    % 핀 윤곽선 (XZ 평면 투영)
    x = [finFront, finFront+rootChord, finFront+rootChord-sweepLength+tipChord, finFront+sweepLength, finFront];
    z = [bodyRadius*sin(angle), bodyRadius*sin(angle), (bodyRadius+finHeight)*sin(angle), (bodyRadius+finHeight)*sin(angle), bodyRadius*sin(angle)];
    
    % 핀 그리기
    patch(x, z, [0.7 0.2 0.2], 'EdgeColor', 'k');
end

% YZ 평면도 (앞에서 본 모습)
subplot(1,3,3);
hold on;
grid on;
title('YZ 평면도 (앞에서 본 모습)');
ylabel('Y (m)');
zlabel('Z (m)');
axis equal;

% 로켓 본체 원 그리기
th = linspace(0, 2*pi, 50);
plot(bodyRadius*cos(th), bodyRadius*sin(th), 'k-', 'LineWidth', 1.5);

% 핀 CG 그리기
scatter(finCGs(:,2), finCGs(:,3), 50, 'ro', 'filled');

% 핀셋 CG 그리기
scatter(finSetCG(2), finSetCG(3), 100, 'go', 'filled');

% 각 핀 부착 방향선 그리기
for i = 1:length(finIndices)
    cg = finCGs(i,:);
    
    % 핀 부착점에서 CG까지 선 그리기
    line([0, cg(2)], [0, cg(3)], 'Color', 'k', 'LineWidth', 1.5);
    
    % 핀 각도 표시 (텍스트)
    angle = finAngles(i);
    text(cg(2)*1.1, cg(3)*1.1, sprintf('%.0f°', rad2deg(angle)), 'FontSize', 8);
end

% Y-Z 평면에서 각도 방향 표시
for angle = 0:45:315
    rad = bodyRadius * 1.2;
    x = rad * cosd(angle);
    y = rad * sind(angle);
    text(x, y, sprintf('%d°', angle), 'HorizontalAlignment', 'center', 'FontSize', 8);
end

% 요약 정보 텍스트
infoText = sprintf(['핀셋 "%s"\n', ...
                  '총 질량: %.4f kg\n', ...
                  'CG 위치: [%.4f, %.4f, %.4f] m\n', ...
                  'Ixx: %.6f\nIyy: %.6f\nIzz: %.6f\n', ...
                  'Δ(Iyy-Izz): %.9f'], ...
                  finSetName, totalMass, ...
                  finSetCG(1), finSetCG(2), finSetCG(3), ...
                  finInertias(1,1,1), finInertias(2,2,1), finInertias(3,3,1), ...
                  abs(finInertias(2,2,1) - finInertias(3,3,1)));

% 정보 텍스트 위치 설정
annotation('textbox', [0.02, 0.02, 0.3, 0.1], 'String', infoText, ...
           'FitBoxToText', 'on', 'BackgroundColor', 'white', 'EdgeColor', 'k');
end