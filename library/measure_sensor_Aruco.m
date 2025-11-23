function [z_measured, z_true] = measure_sensor_Aruco(Xk, X_target, Rk)

    z_measured = zeros(2, 1);

    RI2B = [ cos( Xk(5) ), sin( Xk(5) ); -sin( Xk(5) ), cos( Xk(5) ) ];

    rho_vec_I = X_target( 1:2 ) - Xk( 1:2 );
    rho_vec_B = RI2B * rho_vec_I;

    % Sat to Target (ρ) Measurement
    dx = rho_vec_B(1);
    dy = rho_vec_B(2);
    true_dist = sqrt( dx^2 + dy^2 );

    % disp(true_dist)
    z_measured(1) = true_dist + randn * sqrt( Rk(1,1) );

    true_rel_ang = atan2( dy, dx );
    true_rel_ang = wrapToPi( true_rel_ang );

    z_measured(2) = true_rel_ang + randn * sqrt( Rk(2,2) );
    z_measured(2) = wrapToPi( z_measured(2) );

    z_true = [true_dist; true_rel_ang];
end