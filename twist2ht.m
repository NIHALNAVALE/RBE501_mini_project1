function T = twist2ht(S,theta)
    omega = S(1:3);
    omega_skew = [0 -omega(3) omega(2); omega(3) 0 -omega(1); -omega(2) omega(1) 0] ;
    % If needed, you can calculate a rotation matrix with:
    R = axisangle2rot(omega,theta);
    V = S(4:6);
    
    k = ( eye(3).*theta + (1-cos(theta)).*omega_skew + (theta - sin(theta)).*(omega_skew^2) )*V;
    T = [R k; 0 0 0 1];
end