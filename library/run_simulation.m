function [error, state_history, X_filt_hist] = run_simulation(qs_u, qs_v, qs_r, params)
    % 프로세스 노이즈 시그마 값을 설정
    params.qs_u = qs_u;
    params.qs_v = qs_v;
    params.qs_r = qs_r;

    % 시뮬레이션 실행 (기존 메인 시뮬레이션 코드를 여기에 포함)
    % Add Library directory
    addpath(genpath('C:\Users\DDHD\Desktop\sat_hw_ver_multi3'));
    
    % Initialize setting
    params = Params_init(); % Load Parameter struct
    
    X = [ 517; 484; 0; 0; 0; 0 ]; % ( posX, posY, V_X, V_Y, rotZ, rateZ )
    Master_point = [ 2400; 2400; 0; 0; 0; 0 ]; % Initialize Mothership position and orientation
    
    Rk = params.EKF.Rm;
    
    Q_Fx = 0;
    Q_Fy = 0;
    Q_Tau = 0;
    Qw = diag( [ Q_Fx.^2, Q_Fx.^2, Q_Tau.^2 ]' );
    
    % Simulation setting
    Main_Hz = params.System.MainLoopHz;
    Sensor_VI_Hz = params.Sensor.VICON_HZ;
    PID_control_Hz = Main_Hz;
    OPENCV_HZ = params.Sensor.OPENCV_HZ;
    
    sim_dt = 1 / Main_Hz; % [sec]
    sim_Time = 60; % [sec]
    sim_step = ceil( sim_Time / sim_dt );
    
    % Sensor & Control interval Setting
    Sensor_VI_interval = Main_Hz / Main_Hz; % sensor sampling set up
    PID_control_interval = Main_Hz / PID_control_Hz; 
     
    % Time duty initialize
    real_time = 0;
    
    % Data log initialize
    state_history = zeros(6, sim_step);
    Masterpoint_history = zeros(6, sim_step);
    temp3sigma_EKF = zeros(6, sim_step);
    X_raw_hist = zeros(6, sim_step);
    X_predict_hist = zeros(6, sim_step);
    debug_control = zeros(9, sim_step);
    t = zeros(1, sim_step);
    target_history = zeros(6, sim_step);
    debug_cmd_delta_hist = zeros(9, sim_step);
    X_measure = zeros(6, sim_step);
    X_filt_hist = zeros(6, sim_step);
    z_true_hist = zeros(3, sim_step);
    state_filter_hist = zeros(3, sim_step);
    z_measurement_hist = zeros(3, sim_step);
    
    
    % Main simulation loop
    for i = 1 : sim_step
        
        X_target = generate_commands(real_time); % Generate - X_target & tolerance
    
    
        if mod(i-1, PID_control_interval) == 0 
    
            U = Controller_PID_Argument( X, X_target, sim_dt, params );
    
        end
    
        [ final_cmd, debug_cmd_delta ]  = Control_Allocator( U, params );
    
        % Stochastic RK4 simulation
        X = srk4( @vehicle_dynamics_Airbearing_stochastic, X, final_cmd, Qw, sim_dt, params, sim_dt );
    
        % data save
        state_history(:, i) = X;
        Masterpoint_history(:, i) = Master_point;
        debug_control(:, i) = final_cmd;
        t(i) = real_time;
        target_history(:, i) = X_target;
    
        real_time = real_time + sim_dt;
    end
        
    X_filt = state_history( :, 1 );
    P_ekf =  params.EKF.P;
    
    for k = 1 : sim_step
    
        if mod(k-1, 1/OPENCV_HZ) == 0 
    
            X_ture = state_history( :, k );
            U_ture = debug_control( :, k );
            X_target_ekf = target_history( :, k );
            master_point_current = Masterpoint_history( :, k );
    
            [ z_AHRS, z_AHRS_true ] = measure_sensor_AHRS( X_ture, master_point_current, Rk );
            [ z_Aruco, z_Aruco_true ] = measure_sensor_Aruco( X_ture, master_point_current, Rk );
    
            z = [ z_Aruco; z_AHRS ];
            z_true = [ z_Aruco_true; z_AHRS_true ];
    
            [ X_filt, P_ekf, y_hat, X_predict ] = processDataSIM_Muti( z, P_ekf, X_filt, master_point_current, U_ture, params, sim_dt );
            
            X_rel_I = Master_point(1:2) - X_filt(1:2);
            X_rel_B = [ cos( X_filt(5) ), -sin( X_filt(5) ); sin( X_filt(5) ), cos( X_filt(5) ) ] * X_rel_I;
            rho_k = sqrt( X_rel_B(1)^2 + X_rel_B(2)^2 );
            theta_k = atan2( X_rel_B(2), X_rel_B(2) ) -  X_filt(5);
            r_k = X_filt(6);
    
            % Measurement vector
            h1 = [ rho_k; theta_k; r_k ];
    
        end
    
        X_filt_hist( :, k ) = X_filt;
        temp3sigma_EKF( :, k ) = 3 * sqrt( diag( P_ekf ) );
        X_predict_hist( :, k ) = X_predict;
        z_measurement_hist( :, k ) = z;
        z_true_hist( :, k ) = z_true;
        state_filter_hist( : , k ) = h1;
    
        
        % X_raw_hist( :, k ) = y_hat;
    
    end

    % 추정 오차 계산
    error = calculate_rmse(state_history, X_filt_hist);
    
end
