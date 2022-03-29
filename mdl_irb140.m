%MDL_IRB140 Create model of ABB IRB 140 manipulator
%
% MDL_IRB140 is a script that creates the workspace variable irb140 which
% describes the kinematic characteristics of an ABB IRB 140 manipulator
% using standard DH conventions.
%
% Also define the workspace vectors:
%   qz         zero joint angle configuration
%   qr         vertical 'READY' configuration
%   qd         lower arm horizontal as per data sheet
%
% Reference::
% - "IRB 140 data sheet", ABB Robotics.
% - "Utilizing the Functional Work Space Evaluation Tool for Assessing a 
%   System Design and Reconfiguration Alternatives"
%   A. Djuric and R. J. Urbanic
%
% Notes::
% - SI units of metres are used.
% - Unlike most other mdl_xxx scripts this one is actually a function that
%   behaves like a script and writes to the global workspace.
%
% See also mdl_fanuc10l, mdl_m16, mdl_motormanHP6, mdl_S4ABB2p8, mdl_puma560, SerialLink.

% MODEL: ABB, IRB140, 6DOF, standard_DH



% Copyright (C) 1993-2017, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com
% function q = ikin(S,M, currentQ, targetPose)
    
    


function r = mdl_irb140()
    
    deg = pi/180;
    plotOn = false;

    % Joint limits
    qlim = [-pi/2  pi/2;  % q(1)
            -pi/4  pi/2;  % q(2)
            0      pi/3;  % q(3)
            -pi/2  pi/2;  % q(4)
            -pi/2  pi/2;  % q(5)
            -pi/2  pi/2]; % q(6)

    % robot length values (metres)
    d1 = 0.352;
    a1 = 0.070;
    a2 = 0.360;
    d4 = 0.380;
    d6 = 0.065;
    
    % DH parameter table
    %     theta d a alpha
    dh = [0 d1 a1  -pi/2
          0 0  a2  0
          0 0  0   pi/2
          0 d4 0   -pi/2
          0 0  0   pi/2
          0 d6 0   pi/2];

    % home configuration M
    M = [1 0 0 0.430;
        0 0 -1 0;
        0 1 0 0.797;
        0 0 0 1];

    % screws
    s1 = [0 0 1 0 0 0]';
    s2 = [0 1 0 -0.352 0 0.070]';
    s3 = [0 1 0 -0.352 0 0.430]';
    s4 = [0 0 1 0 -0.430 0]';
    s5 = [0 1 0 -0.732 0 0.430]';
    s6 = [0 0 1 0 -0.430 0]';

    S = [s1,s2,s3,s4,s5,s6]

        robot = SerialLink(dh, 'name', 'IRB 140', ...
        'manufacturer', 'ABB', 'ikine', 'nooffset');

fprintf('---------------------Forward Kinematics Test---------------------\n');
% fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%'); 

% % Test the forward kinematics for 100 random sets of joint variables
    nTests = 20;

    for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    

    % Generate a random configuration
     q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
         qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
         qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(), ...
         qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(), ...
         qlim(5,1) + (qlim(5,2) - qlim(5,1)) * rand(), ...
         qlim(6,1) + (qlim(6,2) - qlim(6,1)) * rand()];

    T = fkine(S,M,q);

    if plotOn
        irb140.teach(q);
        title('Forward Kinematics Test');
    end
    
    assert(all(all(abs(double(robot.fkine(q)) - T) < 1e-10)));
end

fprintf('\nTest passed successfully.\n');





% %% Part C - Calculate the Space Jacobian of the manipulator
fprintf('-------------------Differential Kinematics Test------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%'); 
% 
% % Test the correctness of the Jacobian for 100 random sets of joint
% % variables
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Generate a random configuration
    q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
         qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
         qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(),...
         qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(),...
         qlim(5,1) + (qlim(5,2) - qlim(5,1)) * rand(),...
         qlim(6,1) + (qlim(6,2) - qlim(6,1)) * rand()];
    
    % Calculate the Forward Kinematics
    T = fkine(S,M,q);
    
    
    % Calculate the Jacobian
    J = jacob0(S,q);
    
    if plotOn
        robot.teach(q);
        title('Differential Kinematics Test');
    end
    
    % Test the correctness of the Jacobian
    Jcoords = [-skew(T(1:3,4))*J(1:3,:)+J(4:6,:); J(1:3,:)];
    assert(all(all(abs(double(robot.jacob0(q)) - Jcoords) < 1e-10)));
end

fprintf('\n Jacobian Test passed successfully.\n');
%-------------------------------------------------------------------------------------------------------------------------------------------




%% Part D - Inverse Kinematics
fprintf('----------------------Inverse Kinematics Test--------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%');

% getting the path from make_path.m

path = make_path('infinity');
scatter3(path(1,:), path(2,:), path(3,:), 'filled');

    % Generate the robot's pose
targetPose = zeros(6,size(path,2));

for ii = 1:size(path,2)
    R = M(1:3,1:3);
%     R = [0 0 -1; 0 1 0; 1 0 0]';
    T = [R path(:,ii); 0 0 0 1];

%     disp(size(T))

    targetpose = MatrixLog6(T);

%     disp(size(targetPose))
    targetPose(:,ii) = [targetpose(3,2) targetpose(1,3) targetpose(2,1) targetpose(1:3,4)']';
end

% Calculate the twist representing the robot's home pose
currentPose = MatrixLog6(M);
currentPose = [currentPose(3,2) currentPose(1,3) currentPose(2,1) currentPose(1:3,4)']';

% Set the current joint variables
currentQ = zeros(1,6);

if plotOn
    robot.teach(currentQ);
    h = triad('matrix', M, 'tag', 'Target Pose', 'linewidth', 2.5, 'scale', 0.5);
end
     
% Generate the test configurations
% q = [linspace(0,pi/2,nTests);
%      linspace(0,pi/6,nTests);
%      linspace(0,pi/6,nTests)];

qlist = [];

for ii = 1 : size(path,2)
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/size(path,2)*100));
    
    % Generate the robot's pose
%     
%     targetPose = zeros(6,size(path,2));
%     targetPose = MatrixLog6(T);
%     targetPose = [targetPose(3,2) targetPose(1,3) targetPose(2,1) targetPose(1:3,4)']';
    
    if plotOn
        set(h, 'matrix', T);
        title('Inverse Kinematics Test');
        drawnow;
    end
    
%     T = fkine(S,M,currentQ);
%     currentPose = MatrixLog6(T);
%     
%     currentPose = [currentPose(3,2) ...
%     currentPose(1,3) ...
%     currentPose(2,1) ...
%     currentPose(1:3,4)']';
 
    qlist(ii,1:6) = ikin(S, M, currentQ, targetPose(:,ii));
        
%         if plotOn
%             try
%                 robot.teach(currentQ);
%                 drawnow;
%             catch e
%                 continue;
%             end
%         end
%     end
end

fprintf('\nTest passed successfully.\n');
% and build a serial link manipulator
    
%     robot = SerialLink(dh, 'name', 'IRB 140', ...
%          'manufacturer', 'ABB', 'ikine', 'nooffset'); 
    robot.teach(qlist)
    % place the variables into the global workspace
    if nargin == 1
        r = robot;
    elseif nargin == 0
        assignin('caller', 'irb140', robot);
        assignin('caller', 'qz', [0 0 0 0 0 0]); % zero angles
        assignin('caller', 'qd', [0 -90 180 0 0 -90]*deg); % data sheet pose, horizontal
        assignin('caller', 'qr', [0 -90 90 0 90 -90]*deg); % ready pose, arm up
    end
end

