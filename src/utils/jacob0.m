function J = jacob0(S,q)
    %DESCRIPTION:
    %JACOB0 is a function used to calculate the jacobian for a particular
    %configuration of the robot.
    %INPUT: a 6xN screw axis matrix of a robot, joint angles
    %OUTPUT: Jacobian matrix of the configuration
    J = [];
    T = eye(4);
    for i = 1:length(q)
        T = T * twist2ht(S(:,i),q(i));
        adj_s = adjoint(S(:,i),T);
        J = [J adj_s];
    end

   
end