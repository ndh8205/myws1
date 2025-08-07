function [bias_dot, none4] = bias_dynamics_HANul(bias_10, none1, w, none3, params, none2)
 
    %----------------------------------------------------------------------
    % First order Gauss–Markov(FOGM)
    %
    %   dot(bias) = -1/tau * bias + w
    %   none = 기존 라이브러리를 이용하기 위한 더미값
    %----------------------------------------------------------------------
    
    tau_gyro = params.bias.tau_gyro;
    tau_acc  = params.bias.tau_acc;
    tau_mag  = params.bias.tau_mag;
    tau_baro = params.bias.tau_baro;

    none4 = 0;

    b_gyro  = bias_10(1:3);
    b_acc = bias_10(4:6);
    b_mag  = bias_10(7:9);
    b_baro =  bias_10(10);

    wGyro = w(1:3); 
    wAcc  = w(4:6);
    wMag  = w(7:9);
    wbaro = w(10);

    bGyro_dot = - (1/tau_gyro) .* b_gyro + wGyro;
    bAcc_dot  = - (1/tau_acc)  .* b_acc  + wAcc;
    bMag_dot  = - (1/tau_mag)  .* b_mag  + wMag;
    bbaro_dot = - (1/tau_baro)  .* b_baro  + wbaro;

    bias_dot  = [bGyro_dot; bAcc_dot; bMag_dot; bbaro_dot];

end