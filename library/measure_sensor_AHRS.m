function [ z_measured, z_true ] = measure_sensor_AHRS( Xk, X_target, Rk )
    
    % 위성의 각속도 (r) 측정
    z_measured = Xk(6) + randn * sqrt(Rk(3,3));
    z_true = Xk(6);
end