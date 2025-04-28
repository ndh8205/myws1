function h = measurement_model_Linear_M( xhat, X_target )

    H = calcHMatrix_Linear_M(xhat, X_target);
    h = H * xhat;

end