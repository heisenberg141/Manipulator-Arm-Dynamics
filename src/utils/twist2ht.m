function T = twist2ht(S,theta)
    %DESCRIPTION:
    %TWIST2HT converts a screw axis and theta to transformation matrix
    %INPUT: Screw axis S, theta
    %OUTPUT: transformation matrix T
    omega=S(1:3);
    velocity=S(4:6);
    
    T11= axisangle2rot(omega,theta);
    
    omega_skew=skew(omega);
    T12=(eye(3).*theta + (1 - cos(theta)).* omega_skew + (theta- sin(theta)).*(omega_skew)^2)*velocity;
    
    T=[T11 T12 
        zeros(1,3) 1];
    % If needed, you can calculate a rotation matrix with:
    % R = axisangle2rot(omega,theta);
end
