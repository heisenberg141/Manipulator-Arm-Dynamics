function R = axisangle2rot(omega,theta)
    %DESCRIPTION:
    %AXISANGLE2ROT calculates rotation matrix using axis angle notation
    %INPUT: axis of rotation (omega) and angle of rotation (theta)
    %OUTPUT: rotation matrix R
    omega_skew=skew(omega);
    R= eye(3) +sin(theta).* omega_skew+ (1-cos(theta)).*omega_skew^2 ;  
    
end
