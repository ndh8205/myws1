% 측정 모델 함수 정의
function h = measurement_model_VI(x, X_target)

    h = [ x(1); x(2); x(5) ];
end