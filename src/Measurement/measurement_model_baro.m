function [ baro_meas, baro_bias, baro_true ] = measurement_model_baro(X, R, params)
%--------------------------------------------------------------------------
% measurement_model_baro
%
%  - X(3) : NED에서 D방향 (고도와 반대 부호)
%  - X(23): 바로미터 바이어스(가정)
%  - R    : 센서 잡음 분산 (스칼라)
%  - params
%
% 출력:
%  baro_meas
%  baro_true 
%--------------------------------------------------------------------------

    baro_true   = -X(3);

    baro_noise = sqrt(R(10,10))*randn(1,1);

    baro_bias  = baro_true + X(23);
    baro_meas  = baro_true + X(23) + baro_noise;

end
