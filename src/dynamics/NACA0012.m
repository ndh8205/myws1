clear all
clc
close all

% Define alpha values (x-axis)
alpha = -10:0.1:15;  % From -10 to 15 degrees, with 0.1 degree steps

% Define key points for Cl
Cl_points = [-10, -0.85; -8, -0.88; -6, -0.7; -4, -0.5; -2, -0.25; 
             -1, -0.1; 0, 0; 1, 0.1; 2, 0.25; 4, 0.5; 6, 0.7; 
             8, 0.82; 10, 0.87; 11, 0.85; 12, 0.8; 15, 0.67];

% Define key points for Cd
Cd_points = [-10, 0.075; -8, 0.05; -6, 0.035; -4, 0.025; -2, 0.0215; 
             0, 0.021; 2, 0.0215; 4, 0.023; 6, 0.028; 8, 0.04; 
             10, 0.09; 11, 0.11; 12, 0.105; 15, 0.095];

% Interpolate Cl and Cd values
Cl = interp1(Cl_points(:,1), Cl_points(:,2), alpha, 'linear');
Cd = interp1(Cd_points(:,1), Cd_points(:,2), alpha, 'linear');

% Create the plots
figure;

% Cl vs Alpha plot
% subplot(2,1,1);
plot(alpha, Cl, 'b-', 'LineWidth', 2);
title('Cl v Alpha');
xlabel('Alpha (degrees)');
ylabel('Cl');
grid on;
xlim([-10 15]);
ylim([-1 1]);

% Cd vs Alpha plot
% subplot(2,1,2);
figure;
plot(alpha, Cd, 'b-', 'LineWidth', 2);
title('Cd v Alpha');
xlabel('Alpha (degrees)');
ylabel('Cd');
grid on;
xlim([-10 15]);
ylim([0 0.12]);

% To use these in your simulation, you can use interp1 function:
% Cl_interp = interp1(alpha, Cl, alpha_current, 'linear', 'extrap');
% Cd_interp = interp1(alpha, Cd, alpha_current, 'linear', 'extrap');