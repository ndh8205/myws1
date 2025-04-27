function R_B2I_quat = GetDCM_QUAT(q) % Quaternion DCM Body to Inertial

    % Normalize the quaternion
    q = q / norm(q);
    
    qw = q(1);
    qx = q(2);
    qy = q(3);
    qz = q(4);
    
    R = zeros(3,3);
    R(1,1) = qw^2 + qx^2 - qy^2 - qz^2;
    R(1,2) = 2*(qx*qy - qw*qz);
    R(1,3) = 2*(qx*qz + qw*qy);

    R(2,1) = 2*(qx*qy + qw*qz);
    R(2,2) = qw^2 - qx^2 + qy^2 - qz^2;
    R(2,3) = 2*(qy*qz - qw*qx);

    R(3,1) = 2*(qx*qz - qw*qy);
    R(3,2) = 2*(qy*qz + qw*qx);
    R(3,3) = qw^2 - qx^2 - qy^2 + qz^2;

    R_B2I_quat = R;

end



