function aero = generate_aero_coefficients()

    % Define alpha values (x-axis)
    alpha = -15:0.1:15; % From -10 to 15 degrees, with 0.1 degree steps

    % Define key points for Cl
    Cl_points = [-10, -0.85; -8, -0.88; -6, -0.7; -4, -0.5; -2, -0.25;
                 -1, -0.1; 0, 0; 1, 0.1; 2, 0.25; 4, 0.5; 6, 0.7;
                 8, 0.82; 10, 1; 11, 0.8; 12, NaN; 15, NaN];

    % Define key points for Cd
    Cd_points = [-13, 0.138; -10, 0.055; -6, 0.012; -4, 0.013; -2, 0.0215;
                 0, 0.022; 2, 0.0215; 4, 0.013; 6, 0.012; 8, 0.035;
                 10, 0.055; 11, 0.11; 12, NaN; 15, NaN];

    % Interpolate Cl and Cd values
    Cl = interp1(Cl_points(:,1), Cl_points(:,2), alpha, 'linear');
    Cd = interp1(Cd_points(:,1), Cd_points(:,2), alpha, 'linear');

    % Create the plots
    % figure;
    % plot(alpha, Cl, 'b-', 'LineWidth', 2);
    % title('Cl v Alpha');
    % xlabel('Alpha (degrees)');
    % ylabel('Cl');
    % grid on;
    % xlim([-10 15]);
    % ylim([-1 1]);
    % saveas(gcf, 'Cl_vs_Alpha.png');

    % figure;
    % plot(alpha, Cd, 'b-', 'LineWidth', 2);
    % title('Cd v Alpha');
    % xlabel('Alpha (degrees)');
    % ylabel('Cd');
    % grid on;
    % xlim([-15 15]);
    % ylim([0 0.15]);
    % saveas(gcf, 'Cd_vs_Alpha.png');

    % Save data to CSV file
    data = [alpha', Cl', Cd'];
    csvwrite('aero_coefficients.csv', data);

    % Create aero structure with interpolation functions
    aero.alpha = alpha;
    aero.Cl = Cl;
    aero.Cd = Cd;
    aero.Cl_func = @(a) interp1(alpha, Cl, a, 'linear', 'extrap');
    aero.Cd_func = @(a) interp1(alpha, Cd, a, 'linear', 'extrap');
    
end