function omega_b = angvelocityspace2body(omega_s,R)
    %DESCRIPTION:
    %ANLEVELOCITYSPACE2BODY calculates angular velocity in the body frame
    %given angular velocity in the world frame, and the transformation 
    %matrix between body and world frame
    %INPUT: omega in the world frame (omega_s), transformation matrix T
    %OUTPUT: omega in the body frame
    omega_b = R'* omega_s;
end
