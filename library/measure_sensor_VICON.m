function z_measured = measure_sensor_VICON( Xk, X_target, Rk )

    z_measured = Xk + randn( 6, 1 ) .* [ Rk(1,1), Rk(2,2), Rk(3,3), Rk(4,4), Rk(5,5), Rk(6,6) ]';

end