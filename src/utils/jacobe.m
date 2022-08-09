function J_b = jacobe(S,M,q)    
    %DESCRIPTION:
    %JACOBe is a function used to calculate the jacobian for a particular
    %configuration of the robot in the body frame.
    %INPUT: a 6xN screw axis matrix of a robot, joint angles, home
    %configuration
    %OUTPUT: Jacobian matrix of the configuration in the body frame.
    n = size(S);
    H = eye(4);
    for i = 1:n(2)
        T = twist2ht(S(:,i),q(i));
        H = H*T;
        J(:,i) = adjoint(S(:,i),H);
    end
    
    T = fkine(S,M,q,'space');
    R = T(1:3,1:3);
    p = T(1:3,4);
    P_brack = skew_sym(p);
    adT = [R zeros(3,3); P_brack*R R];
    
    J_b = inv(adT)*J;
    
end