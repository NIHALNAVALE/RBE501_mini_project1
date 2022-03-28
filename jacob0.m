function J = jacob0(S,q) 
    len = width(S);
    I = eye(4);
    
    for i = 1:len
        H = twist2ht(S(:,i),q(i));
%        H = twist2ht(V,omega)
        I = I*H;
        adM = adjoint(S(:,i), I);
        J(:,i) = adM;
    end
end