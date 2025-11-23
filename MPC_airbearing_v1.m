clc,clear
close all

%% state space model
% airbearing
I_Z =  453247; % [kgmm^2]
I_Z =  900000; % [kgmm^2]

d = 95; % [mm] (CG to Thruster) 
m = 19; % [kg] (Airbearing mass)
T = 70; % [N] (Mean Thrust Force)


x0 = [0 0 0 0 0 0]'; % 초기값
rr = [1000 1000 0 0 deg2rad(10) 0]'; % 목표값
u = [0 0 0 0 0 0 0 0]'; % u(k-1) =0

psi = x0(5);
u0 = x0(3);
v0 = x0(4);
r0 = x0(6);

Ac = [0 0 cos(psi) -sin(psi) -u0*sin(psi)-v0*cos(psi) 0;
      0 0 sin(psi)  cos(psi)  u0*cos(psi)-v0*sin(psi) 0;
      0 0    0       -r0             0               -v0;
      0 0   r0         0             0                u0;
      0 0    0         0             0                1;
      0 0    0         0             0                0];

Bc = [0      0      0      0      0      0      0      0;
      0      0      0      0      0      0      0      0;
      0    -T/m   -T/m     0      0     T/m    T/m     0;
    -T/m     0      0     T/m    T/m     0      0    -T/m;
      0      0      0      0      0      0      0      0;
    -T*d/I_Z   T*d/I_Z   -T*d/I_Z    T*d/I_Z   -T*d/I_Z    T*d/I_Z   -T*d/I_Z    T*d/I_Z];

% B = [0      0      0      0      0      0      0      0;
%      0      0      0      0      0      0      0      0;
%      0    -T/m   -T/m     0      0     T/m    T/m     0;
%    -T/m     0      0     T/m    T/m     0      0    -T/m;
%      0      0      0      0      0      0      0      0;
%     -T*d/Iz T*d/Iz -T*d/Iz T*d/Iz -T*d/Iz T*d/Iz -T*d/Iz T*d/Iz];

% Cc = [1 0 0 0;
%       0 1 0 0;
%       0 0 1 0;
%       0 0 0 1];

Cc = eye(6);

Dc = zeros(length(Cc(:, 1)), length(Bc(1, :)));

% 역진자
% M = 2.4;
% m = 0.23;
% l = 0.4;
% g = 9.8;
% 
% Ac = [0 1 0 0;
%       (M+m)*g/(M*l) 0 0 0;
%       0 0 0 1;
%       -(m*g)/M 0 0 0];
% 
% Bc = [0;
%       -1/(M*l);
%       0;
%       1/M];
% 
% Cc = [1 0 0 0;
%       0 0 1 0];
% 
% % Cc = [1 0 0 0;
% %       0 1 0 0;
% %       0 0 1 0;
% %       0 0 0 1];
% 
% Dc = zeros(length(Cc(:, 1)), length(Bc(1, :)));
% 
% x0 = [0.2 0 0 0]'; % 초기값
% rr = [0 0]'; % 목표값
% u = 0; % u(k-1) =0

% TH Eq
% 
% mu = 398600*10^9; % m^3/s^2
% % a = 12000000; % m
% a = 6378000+700000; % m
% e = 0.1;
% theta = 0;
% theta_dot = (1+e*cos(theta))^2/((1-e^2)^(3/2))*sqrt(mu/a^3);
% theta_2dot = -2*(mu*e*sin(theta)*(1+e*cos(theta))^3)/(a^3*(1-e^2)^3);
% 
% Ac = [0 0 0 1 0 0;
%        0 0 0 0 1 0;
%        0 0 0 0 0 1;
%        (3+e*cos(theta))*theta_dot^2/(1+e*cos(theta)) theta_2dot 0 0 2*theta_dot 0;
%        -theta_2dot e*cos(theta)*(theta_dot^2)/(1+e*cos(theta)) 0 -2*theta_dot 0 0;
%        0 0 -(theta_dot^2)/(1+e*cos(theta)) 0 0 0];
% 
% Bc = [0 0 0;
%        0 0 0;
%        0 0 0;
%        1 0 0;
%        0 1 0;
%        0 0 1];
% 
% Cc = [1 0 0 0 0 0;
%        0 1 0 0 0 0;
%        0 0 1 0 0 0];
% 
% Dc = zeros(length(Cc(:, 1)), length(Bc(1, :)));
% 
% 
% x0 = [30; -30; 0; -0.5; -0.5; 0]; % 초기값
% rr = [0; -1; 0]; % 목표값
% u = zeros(size(Bc,2),1);; % u(k-1) =0

%% parameter
Np = 100;
Nc = 10;
dt = 0.0625;

[Ad, Bd, Cd, Dd] = c2dm(Ac, Bc, Cc, Dc, dt);

N_sim = 40;

q = eye(size(Cd,1))*1000; % weight on tracking error
r = eye(size(Bd,2))*1; % weight on control chnage

% Ad=[1 1;0 1];
% Bd=[0.5;1];
% Cd=[1 0];
% Dd=0;

[n,n] = size(Ad);
[n,m] = size(Bd);
[p,n] = size(Cd);

A_a = eye(n+p,n+p);
A_a(1:n,1:n) = Ad;
A_a(n+1:n+p,1:n) = Cd*Ad;

B_a = zeros(n+p,m);
B_a(1:n,:) = Bd;
B_a(n+1:n+p,:) = Cd*Bd;

C_a = zeros(p,n+p);
C_a(:,n+1:n+p) = eye(p,p);


%% Loop to create each block of the W matrix
W = [];

W_1 = C_a*A_a;

for i = 1:Np
    W_block = W_1 * (A_a^(i-1));
    W = [W; W_block];
end

%% Construct the Z matrix
Z = zeros(p*Np,m*Nc); %declare the dimension of Phi

for i = 1:Np
    for j = 1:Nc
        if i >= j
            Z((i-1)*p+1:i*p, (j-1)*m+1:j*m) = C_a * A_a^(i-j) * B_a;
        else
            Z((i-1)*p+1:i*p, (j-1)*m+1:j*m) = zeros(p, m);
        end
    end
end

%% 가중치 및 목표값 행렬 생성
Q = []; R = []; Rr = [];

for i = 1:Np
  Q = blkdiag(Q,q);
  Rr = [Rr;rr];
end

for i = 1:Nc
  R = blkdiag(R,r);
end

%% MPC

Nc_block = [eye(m,m),zeros(m,m*Nc-m)];

H  = Z'*Q*Z + R;

x1 =  Ad*x0 + Bd*u;
y = Cd*x1;
dx = x1 - x0;

XF = [dx; y];

%% 제약조건

% du_min = -1;
% du_max = 1;

y_min = [-inf;
         -inf
         -40
         -40
         -inf
         -inf];


y_max = [inf;
         inf
         40
         40
         inf
         inf];

% u_min = 0;
% u_max = 1;

% DU_min = du_min*ones(m*Nc,1);
% DU_max = du_max*ones(m*Nc,1);

Y_min = [];
for i = 1:Np
    Y_min((i-1)*p+1 : i*p, 1) = y_min;
end

Y_max = [];
for i = 1:Np
    Y_max((i-1)*p+1 : i*p, 1) = y_max;
end

% U_min = u_min*ones(m*Nc,1);
% U_max = u_max*ones(m*Nc,1);

%% 
% delta U 제약조건
% C1_I = eye(Nc*m);
% C1 = [-C1_I;
%       C1_I];

% Y 제약조건
C2 = [-Z;
      Z];


% U 제약조건
% C3_H = zeros(Nc*m,Nc*m);
% 
% for i = 1:Nc
%     for j = 1:i
%         C2((i-1)*m + 1:i*m, (j-1)*m + 1:j*m) = eye(m);
%     end
% end
% 
% for i = 1:Nc
%     for j = 1:Nc
%         if i >= j
%             C3_H((i-1)*m + 1:i*m, (j-1)*m + 1:j*m) = eye(m);
%         end
%     end
% end
% 
% E = [];
% for i = 1:Nc
%     E((i-1)*m+1 : i*m, 1:m) = eye(m);
% end
% 
% C3 = [-C3_H;
%       C3_H];
% 

%% all con
% Acon = [C1;
%         C2;
%         C3];
% 
% 
% C_min_max = [-DU_min;
%              DU_max;
%              -Y_min + W*XF;
%              Y_max - W*XF;
%              -U_min + E*u;
%              U_max - E*u];

%% DU con
% Acon = [C1;
%         C3];
% 
% C_min_max = [-DU_min;
%              DU_max;
%              -U_min + E*u;
%              U_max - E*u];

%% U con
% Acon = C3;
% 
% C_min_max = [-U_min + E*u;
%              U_max - E*u];

%% Y con
Acon = C2;

C_min_max = [-Y_min + W*XF;
             Y_max - W*XF];


% y1 = []; y2 = []; y3 = []; y4 = []; y5 = []; y6 = [];
% u1 = []; u2 = []; u3 = []; u4 = []; u5 = []; u6 = []; u7 = []; u8 = [];

y1 = [];
u1 = [];
T_T1 = [];

%% propagate

for kk=0.0625:0.0625:N_sim

    tic
    
    f  = Z'*Q*(W*XF-Rr);

    DeltaU = quadprog(H,f,Acon,C_min_max);
    % DeltaU = quadprog(H,f);

    deltau = Nc_block*DeltaU;

    u = u + deltau;

    T_T = toc;
    T_T1= [T_T1;T_T];

    % for i = 1:m
    %     if deltau(i) > 0
    %         u(i) = 1;
    %     else
    %         u(i) = 0;
    %     end
    % end

    for i = 1:m
        if u(i) >= mean(u)
            u(i) = 1;
        else
            u(i) = 0;
        end
    end

    % for i = 1:m
    %     if u(i) > 0
    %         u(i) = 1;
    %     else
    %         u(i) = 0;
    %     end
    % end

    % 값 저장
    y1 = [y1;y'];
    u1 = [u1;u'];

    x_old = x0;
    x =  Ad*x0 + Bd*u;
    y = Cd*x;
    
    XF=[x-x_old;y];   

    x0 = x;


    % C_min_max = [-DU_min;
    %              DU_max;
    %              -U_min + E*u;
    %              U_max - E*u];

    % C_min_max = [-U_min + E*u;
    %              U_max - E*u];

    C_min_max = [-Y_min + W*XF;
                 Y_max - W*XF];

    
    fprintf("time step : %f", kk)

    
end


%% figure

for i = 1:N_sim+1
    ref_x_plot(i) = rr(1);
    ref_y_plot(i) = rr(2);
    ref_theta_plot(i) = rad2deg(rr(5));
end

x0_s = [0 0 0 0 0 0]; % 초기값

y_f = [x0_s; y1];

k1=0:0.0625:N_sim;
figure(1)
subplot(311)
plot(k1,y_f(:,1))
hold on
plot(0:N_sim,ref_x_plot, 'r--')
title('posX')

subplot(312)
plot(k1,y_f(:,2))
hold on
plot(0:N_sim,ref_y_plot, 'r--')
title('posY')

subplot(313)
plot(k1,rad2deg(y_f(:,5)))
hold on
plot(0:N_sim,ref_theta_plot, 'r--')
title('theta')
% plot(k1,y_f(1),y_f(2),y_f(5))
% xlabel('Sampling Instant')
% legend('Output')

figure(2);
k2=0.0625:0.0625:N_sim;
for i = 1:8
    subplot(8, 1, i);
    stairs(k2,u1(:,i))
    % ylim([0, 1]);
    xlabel('[sec]')
    ylabel('[ON-OFF]')
    title(['Relay ' num2str(i)]);
    hold on
end
