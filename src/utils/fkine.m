function T = fkine(S,M,q,frame)
    %DESCRIPTION:
    %FKINE calculates the transformation matrix of a particular robot in
    %the space frame or the body frame.
    %INPUT: a 6xN screw axis matrix of a robot, joint angles, and frame
    %type -- body or space
    %OUTPUT: transformation matrix T 
    T= eye(4);
    n= size(S);
    for i =1:n(2)
        screw_i=S(:,i);
        T=T*twist2ht(screw_i,q(i));          
    end        
    if strcmp(frame, 'body')
        T=M*T;
    elseif strcmp(frame, 'space')
        T=T*M;
    end
    % If needed, you can convert twists to homogeneous transformation matrices with:
    % twist2ht(S(i),q(i));
end