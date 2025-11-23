% 측정 모델 함수 정의
function h = measurement_model(x, X_target)
    % x: 상태 벡터
    % X_target: 목표 위치

    % 상대 위치 계산
    dx = X_target(1) - x(1);
    dy = X_target(2) - x(2);

    % 몸체 좌표계로 변환
    RI2B = [ cos( x(5) ), sin( x(5) ); -sin( x(5) ), cos( x(5) ) ];
    dxy_body = RI2B * [ dx; dy ];

    % rho 계산
    rho = sqrt( dxy_body(1)^2 + dxy_body(2)^2 );

    % theta 계산
    theta = atan2( dy, dx ) - x(5);
    theta = wrapToPi(theta);

    % h(x) 구성
    h = [ rho; theta; x(6) ];
end