function M_fin = calculate_fin_moment( alpha, beta, fin_deflection, V_B, params )

    % Inputs:
    % alpha: angle of attack (radians)
    % beta: sideslip angle (radians)
    % fin_deflection: deflection angle of each fin (radians) [delta_1, delta_2, delta_3, delta_4]
    % V_B: body velocity vector (body frame)
    % params: rocket and fin parameters

    % Fin parameters
    S_fin = params.fin.chord * params.fin.span;  % fin area
    MAC = params.fin.chord;  % mean aerodynamic chord
    num_fins = params.fin.num;
    x_fin = params.fin.position;  % fin aerodynamic center position (distance from CG)

    % Air density
    rho = params.environment.ro;

    % Calculate dynamic pressure
    q = 0.5 * rho * norm(V_B)^2;

    % Approximate lift curve slope for NACA 0012 (2D)
    Cl_alpha_2D = 2 * pi;

    % Lift curve slope considering 3D effects (simple approximation)
    AR = params.fin.span^2 / S_fin;  % aspect ratio
    Cl_alpha_3D = Cl_alpha_2D / (1 + Cl_alpha_2D / (pi * AR));

    % Calculate moment for each fin
    M_fin = zeros(3, 1);

    for i = 1:num_fins
        % Calculate effective angle of attack for each fin
        alpha_eff = alpha + fin_deflection(i);
        
        % Calculate fin lift
        L_fin = q * S_fin * Cl_alpha_3D * alpha_eff;
        
        % Calculate moment arm based on fin position (simplified example, could be more complex in reality)
        r_fin = [x_fin * cos((i-1)*2*pi/num_fins), x_fin * sin((i-1)*2*pi/num_fins), 0];
        
        % Calculate and sum moments
        M_fin = M_fin + cross(r_fin, [0; 0; -L_fin]);  % Lift is in negative z-direction
    end

    % Add yawing moment due to sideslip angle (simplified model)
    Cn_beta = 0;  % Example value, should be determined through aerodynamic analysis or experiment
    M_fin(3) = M_fin(3) + q * S_fin * MAC * Cn_beta * beta;

end