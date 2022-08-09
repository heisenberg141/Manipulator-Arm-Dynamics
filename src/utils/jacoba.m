function J_a = jacoba(S,M,q)    
    %DESCRIPTION:
    %JACOBA is a function used to calculate the analytical jacobian for a 
    %particular configuration of the robot.
    %INPUT: a 6xN screw axis matrix of a robot, joint angles, Home
    %configuration
    %OUTPUT: analytical Jacobian matrix of the configuration
    T = fkine(S,M,q,'space');
    R = T(1:3,1:3);
    J_b = jacobe(S,M,q);
    J_a = R*J_b(4:6,:);
    
end