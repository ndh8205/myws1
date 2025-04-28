function [ X, P, yhat, xhat ] = HANul_EKF( z, P, X, X_target, Ut, params, dt, step )
%==========================================================================
% 6DOF EKF
% Input
% - z         : Observation
% - P         : Covariance
% - X         : State [position, velocity, quaternion, angular rate]
% - X_target  : Command
% - Ut        : Control input (Thurst)
% - params    : Parmeter_struct
% - dt        : Sampling time
%
% Output
%   X     : State(Update)
%   P     : Covariance(Update)
%   yhat  : innovation
%   xhat  : Predict state
%==========================================================================

    F = calcFMatrix(X, dt, params);
    Qk = calcQMatrix(X, dt, params);
    R = calcRMatrix();


    % xhat = stochastic_predict( @vehicle_dynamics_HANul, X, Ut, zeros(13), dt, params, step, dt );
    xhat = predict_rk1( @vehicle_dynamics_HANul, X, Ut, zeros(13), dt, params, step, dt );
    q_pred = xhat(7:10);

    if norm(q_pred) > 1e-12
        disp('break_first')
        xhat(7:10) = q_pred / norm(q_pred);
    end

    P = F * P * F' + Qk;
    h = measurement_model_Linear( xhat, X_target );

    % innovation
    yhat = z - h; % residual
    H = calcHMatrix_Linear(xhat, X_target); 
    S = H * P * H' + R;
    
    % Kalman gain
    K = P * H' * inv(S);

    % Update
    X = xhat + K * yhat;
    P = (eye(size(P)) - K * H) * P * (eye(size(P)) - K * H)' + K * R * K';

end