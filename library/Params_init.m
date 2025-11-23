%%
function params = Params_init()
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     
    % Main System Parameters

    params.System.MainLoopHz = 16; % [Hz] 

    % Jetson 1 - AB_1 (Data collection + Command)
    deviceAddress1 = '192.168.0.101';
    userName1 = 'jetson1';
    password1 = 'rkd2233';
    
    % Jetson 2 - AB_2 (Command only)
    % deviceAddress2 = '192.168.0.83';
    % userName2 = 'lts';
    % password2 = 'rkd2233';

    params.System.relayPins = [ 37, 35, 31, 29, 23, 21, 19, 13 ];

    params.System.hwjetson1 = jetson( deviceAddress1, userName1, password1 );
    % params.System.hwjetson2 = jetson( deviceAddress2, userName2, password2 );

    % Socket setting
    params.System.commandPort = 54322; 
    params.System.host1 = deviceAddress1;  % Jetson 1 (AB_1)
    % params.System.host2 = deviceAddress2;  % Jetson 2 (AB_2)



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
    % Limit Paramters

    params.Airbearing.C1 = 0.2; % [mm]
      
    params.Airbearing.SL1 = 10; % [mm/s]
    params.Airbearing.SL2 = 10; % [mm/s]
    params.Airbearing.SL3 = 10; % [mm/s]
 
    params.Airbearing.C2 = deg2rad(5); % [deg]

    params.Airbearing.SA1 = deg2rad(15);% [deg/s]
    params.Airbearing.SA2 = deg2rad(100); % [deg/s]
    params.Airbearing.SA3 = deg2rad(45); % [deg/s]
  
    params.Airbearing.Kp_pp = 1; % Velocity P

    % Velocity_PID = [U V]

    params.Airbearing.Kp_p = 0.85; % Velocity P
    params.Airbearing.Kp_i = 0; % Velocity I
    params.Airbearing.Kp_d = 0; % Velocity D
 
    % Attitude_PID = [Yaw]

    params.Airbearing.Ka_pp = 0.8;

    params.Airbearing.Ka_p = 0.65; % Attitude P
    params.Airbearing.Ka_i = 0; % Attitude I
    params.Airbearing.Ka_d = 0; % Attitude D

    % Reaction Wheel Parameters

    M_motor = 0.27; % [kg] Motor mass
    R_motor = 42; % [mm] Motor radius
    M_disk = 1; % [kg] Fly wheel mass
    R_disk = 47; % [mm] Fly wheel radius

    I_motor = 0.5 * M_motor * R_motor^2;
    I_disk = 0.5 * M_disk * R_disk^2;
    
    kv_rating = 100; % KV rating of the motor
    battery_voltage = 25.2; % Voltage of 6-cell LiPo battery in volts

    max_rpm = kv_rating * battery_voltage; % Maximum RPM

    
    params.Reaction_wheel.omega_limit = max_rpm * 2 * pi / 60; % Convert RPM to rad/s
    params.Reaction_wheel.I_wheel = I_motor + I_disk; % [kg*mm^2] Reaction wheel moment
    params.Reaction_wheel.omega_wheel_init = 0; % [rad/s] 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     
    % Airbearing Parameters

    params.Airbearing.I_Z = 453247; % [kgmm^2]
    params.Airbearing.D_ref = 95; % [mm] (CG to Thruster) 
    params.Airbearing.m = 19; % [kg] (Airbearing mass)
    params.Airbearing.Thrust = 75; % [mN] (Mean Thrust Force)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Sensor Parameters

    params.Sensor.VICON_HZ = 120;
    params.Sensor.AHRS_HZ = 144;
    params.Sensor.OPENCV_HZ = 144;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
    % EKF Paramters

    q_x = 0;
    q_y = 0;
    q_u = 20;
    q_v = 20;
    q_psi = deg2rad(0);
    q_r = deg2rad(10);

    Rq_x = 0.365;
    Rq_y = 0.365;
    Rq_u = 10;
    Rq_v = 10;
    Rq_psi = deg2rad(0.1);
    Rq_r = deg2rad(10);

    Rq_rho = 5;
    Rq_theta = deg2rad(5);
    Rq_rm = deg2rad(1);

    params.EKF.Rm = diag( [ Rq_rho.^2, Rq_theta.^2, Rq_rm.^2 ] );
    params.EKF.Rv = diag( [ Rq_x.^2, Rq_y.^2, Rq_psi.^2 ] );

    Pq_x = 10.365;
    Pq_y = 10.365;
    Pq_u = 11;
    Pq_v = 11;
    Pq_psi = deg2rad(10.01);
    Pq_r = deg2rad(10.01);

    params.EKF.P = diag( [ Pq_x^2, Pq_y^2, Pq_u^2, Pq_v^2, Pq_psi^2, Pq_r^2 ] );
    params.EKF.Q = diag( [ q_x^2, q_y^2, q_u^2, q_v^2, q_psi^2, q_r^2 ] );
    params.EKF.R = diag( [ Rq_x^2, Rq_y^2, Rq_u^2, Rq_v^2, Rq_psi^2, Rq_r^2 ] );


    % UKF Paramters

    params.UKF.alpha = 1;
    params.UKF.beta = 1;
    params.UKF.kappa = 1;
    
    UPq_x = 10.365;
    UPq_y = 10.365;
    UPq_u = 11;
    UPq_v = 11;
    UPq_psi = deg2rad(0.01);
    UPq_r = deg2rad(0.01);

    Uq_x = 0;
    Uq_y = 0;
    Uq_u = 40;
    Uq_v = 40;
    Uq_psi = deg2rad(0);
    Uq_r = deg2rad(10);

    params.UKF.P = diag( [ UPq_x^2, UPq_y^2, UPq_u^2, UPq_v^2, UPq_psi^2, UPq_r^2 ] );
    params.UKF.Q = diag( [ Uq_x^2, Uq_y^2, Uq_u^2, Uq_v^2, Uq_psi^2, Uq_r^2 ] );
    params.UKF.R = diag( [ Rq_x^2, Rq_y^2, Rq_u^2, Rq_v^2, Rq_psi^2, Rq_r^2 ] );

end