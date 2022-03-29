function q = ikin(S,M,currentQ,targetPose)
% your code here
    T = fkine(S,M,currentQ);
    currentPose = MatrixLog6(T);
    currentPose = [currentPose(3,2) ...
        currentPose(1,3) ...
        currentPose(2,1) ...
        currentPose(1:3,4)']';
% Inverse Kinematics NEWTON-RAPHSON Method
    while norm(targetPose - currentPose) > 1e-3
        J = jacob0(S,currentQ);
       
        deltaQ = pinv(J)*(targetPose - currentPose);
        currentQ = currentQ + deltaQ';
        
        T = fkine(S,M,currentQ);
        currentPose = MatrixLog6(T);
        currentPose = [currentPose(3,2) ...
                       currentPose(1,3) ...
                       currentPose(2,1) ...
                       currentPose(1:3,4)']';
        
    end
    q = currentQ;
end

