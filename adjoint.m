function twist_inB = adjoint(twist_inA,T_AB)
    O = zeros(3);
    R = T_AB(1:3,1:3); % T = [[R] [p]; 0 1];
    p = T_AB(1:3,4);
    p_skew = [0 -p(3) p(2); p(3) 0 -p(1); -p(2) p(1) 0];
    adM = [R O; p_skew*R R];
    
    twist_inB = adM * twist_inA;
end

