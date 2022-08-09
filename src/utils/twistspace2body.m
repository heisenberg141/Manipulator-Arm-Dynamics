function V_b = twistspace2body(V_s,T)
    %DESCRIPTION:
    %TWISTSPACE2BODY converts a twist vector from the space frame to the
    %body frame
    %INPUT: Twist V_s, Transformation matrix T between body and the space
    %frame
    %OUTPUT: twist in the body frame.
    V_b = adjoint(V_s,inv(T));
end