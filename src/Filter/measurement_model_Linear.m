function h = measurement_model_Linear( xhat, X_target )

    H = calcHMatrix_Linear(xhat, X_target);
    h = H * xhat;

end