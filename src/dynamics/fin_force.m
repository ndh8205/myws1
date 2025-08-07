function F_fin = fin_force( alpha, beta, fin_deflection, V_B, params )

    % Air density
    rho = params.environment.ro;

    % Calculate dynamic pressure
    q = 0.5 * rho * norm(V_B)^2;

    for i = 1 : num_fins
        % Calculate effective angle of attack for each fin
        alpha_eff = alpha + fin_deflection(i);
        
        % Calculate fin lift
        L_fin = q * S_fin * Cl * alpha_eff;
        
        % Calculate moment arm based on fin position (simplified example, could be more complex in reality)
        r_fin = [x_fin * cos((i-1)*2*pi/num_fins), x_fin * sin((i-1)*2*pi/num_fins), 0];
        
        % Calculate and sum moments
        M_fin = M_fin + cross(r_fin, [0; 0; -L_fin]);  % Lift is in negative z-direction
    end

    % Add yawing moment due to sideslip angle (simplified model)
    Cn_beta = 0;  % Example value, should be determined through aerodynamic analysis or experiment
    F_fin(3) = M_fin(3) + q * S_fin * MAC * Cn_beta * beta;

end