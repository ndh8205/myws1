function [X, P, yhat, xhat] = HANul_Navigation_filter(z, P, X, X_target, Ut, params, dt, step)
%==================================================================
% 6DOF Multiplicative EKF for HANul Rocket 2024.12.30
% [Input]
%   - z         : Measurement (pos(LLH), Rate(body), acc(body), mag(body))
%   - P         : Error state Covariance (21x21)
%   - X         : Norminal state (22x1 => [px,py,pz,  u,v,w,  qw,qx,qy,qz,  wx,wy,wz, bgx,bgy,bgz, bax,bay,baz, bmx,bmy,bmz])
%   - X_target  : Command_state (13x1)
%   - Ut        : Control input (8x1)
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

F = calcFeskfMatrix(X, dt, params);
Qk = calcQeskfMatrix(X, dt, params);
H = calcHeskfMatrix(X, dt, params);
Rk = calcRMatrix_ahrs_gps_baro(params);

xhat = predict_rk1(@dynamics_Nav, X, Ut, zeros(23,1), dt, params, step, dt);

q_pred = xhat(7:10);
if norm(q_pred) > 1e-12
    xhat(7:10) = q_pred / norm(q_pred);
end

P = F * P * F' + Qk;

h = measurement_model_HANul( xhat, params );
yhat = z - h;

S = H * P * H' + Rk;
K = P * H' * inv(S);

deltaX = K * yhat;

P = (eye(22) - K*H) * P * (eye(22) - K*H)' + K*Rk*K';

% 위치, 속도, 각속도 업데이트 (덧셈)
p_upd    = xhat(1:3) + deltaX(1:3);
v_upd    = xhat(4:6) + deltaX(4:6);
w_upd    = xhat(11:13) + deltaX(10:12);

% 쿼터니언 업데이트 (곱셈)
delta_theta = deltaX(7:9);
q_upd = q2q_mult(sa_quaternion(delta_theta), xhat(7:10));
q_upd = q_upd / norm(q_upd);

% 바이아스 업데이트 (덧셈)
bg_upd = xhat(14:16) + deltaX(13:15);
ba_upd = xhat(17:19) + deltaX(16:18);
bm_upd = xhat(20:22) + deltaX(19:21);
bb_upd = xhat(23) + deltaX(22);

X = [p_upd; v_upd; q_upd; w_upd; bg_upd; ba_upd; bm_upd; bb_upd];

end

function Xdot_23 = dynamics_Nav(X_23, U, Qk, dt, params, t)
   
    X_13  = X_23(1:13);
    bias_10 = X_23(14:23);

    Xdot_13 = vehicle_dynamics_HANul(X_13, U, Qk, dt, params, t);

    bias_dot_10 = bias_dynamics_HANul(bias_10, U, Qk(14:23), dt, params, t);

    Xdot_23 = [Xdot_13; bias_dot_10];
end