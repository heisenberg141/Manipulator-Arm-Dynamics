function q = ikin(S,M,currentQ,targetPose)
% IKIN Solves the inverse kinematics of the robot
%
% Inputs: S - 6xn matrix whose columns are the screw axes of the robot
%         M - homogeneous transformation representing the home configuration
%         currentQ - n-dimensional column vector of joint variables
%         targetPose - 3-dimensional column vector of Cartesian coordinates
%
% Output: q - n-dimensional column vector of joint variables
    
%     norm(currentPose - targetPose)
    T = fkine(S,M,currentQ,'space');
    currentPose = T(1:3,4);
    
    while norm(targetPose - currentPose) > 1e-3
        J_a = jacoba(S,M,currentQ);

        % Use the Levenberg-Marquadt algorithm (Damped Least Squares)
        lambda = 0.01;
        deltaQ = J_a' * pinv(J_a*J_a' + lambda^2 * eye(3)) * (targetPose - currentPose);
        
        currentQ = currentQ + deltaQ;
        
        T = fkine(S,M,currentQ,'space');
        currentPose = T(1:3,4);
    end
    q = currentQ;


end

