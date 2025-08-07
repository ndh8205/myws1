function [X, P, yhat, xhat] = HANul_MEKF(z, P, X, X_target, Ut, params, dt, step)
%==================================================================
% 6DOF Multiplicative EKF for HANul Rocket 2024.12.30
% [Input]
%   - z         : Measurement (Position, Rate)
%   - P         : Error state Covariance (12x12 가정)
%   - X         : Norminal state (13x1 => [px,py,pz,  u,v,w,  qw,qx,qy,qz,  wx,wy,wz])
%   - X_target  : Command_state
%   - Ut        : Control input
%   - params    : Parameters
%   - dt        : Sampling step
%   - step      : Simulation step
%
% [Output]
%   - X         : Update norminal state (13x1)
%   - P         : Update Error state Covariance (12x12)
%   - yhat      : Residual (z - h)
%   - xhat      : Predictive norminal state
%==================================================================

F_mekf = calcFmekfMatrix(X, dt, params);
Q_mekf = calcQmekfMatrix(X, dt, params);

xhat = predict_rk1(@vehicle_dynamics_HANul, X, Ut, zeros(13,1), dt, params, step, dt);

q_pred = xhat(7:10);
if norm(q_pred) > 1e-12
    xhat(7:10) = q_pred / norm(q_pred);
end

P = F_mekf * P * F_mekf' + Q_mekf;

h = measurement_model_Linear(xhat, X_target);
yhat = z - h;

H_mekf = calcHMatrix_mekf(xhat, X_target);
R = calcRMatrix(params);

S = H_mekf * P * H_mekf' + R;
K = P * H_mekf' * inv(S);

deltaX = K * yhat;

P = (eye(12) - K*H_mekf) * P * (eye(12) - K*H_mekf)' + K*R*K';

% 위치, 속도, 각속도 업데이트 (덧셈)
p_upd = xhat(1:3)   + deltaX(1:3);
v_upd = xhat(4:6)   + deltaX(4:6);
w_upd = xhat(11:13) + deltaX(10:12);

% 쿼터니언 업데이트 (곱셈)
delta_theta = deltaX(7:9);
delta_quat  = sa_quaternion(delta_theta);
q_upd       = q2q_mult(delta_quat, xhat(7:10));
q_upd       = q_upd / norm(q_upd);

X = [p_upd; v_upd; q_upd; w_upd];

end
