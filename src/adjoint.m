function Vtrans = adjoint(V,T)
    %DESCRIPTION:
    %ADJOINT calculates the adjoint of a vector with a matrix
    %INPUT: a vector V, Transformation matrix T
    %OUTPUT: adjoint of adjoint of V with T
    R= T(1:3,1:3);
    p= T(1:3,4);
    p_brack= skew(p);
    Adj_matrix=[R zeros(3,3)
            p_brack*R R];
    Vtrans= Adj_matrix*V;
    
    
end