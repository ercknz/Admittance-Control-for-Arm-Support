function [qS, qE, qDotS, qDotE] = armSupportIKine(X,Y,Xdot,Ydot)
%% Arm Support Inverse Kinematics
% This function is used to calculate the joint angles and angular
% velocities of the robot given the position and velocity of the object in
% the workspace.
%
% Script by erick nunez

%% Constants
L1 = 0.510;
L2 = 0.505;

%% Calculation
qE = acos((X^2 + Y^2 - L1^2 - L2^2)/(2*L1*L2));
if Y < 0
    qS = atan2(Y,X) - atan2((L2 * sin(qE)),(L1 + L2 * cos(qE))) + 2*pi;
else
    qS = atan2(Y,X) - atan2((L2 * sin(qE)),(L1 + L2 * cos(qE)));
end
qDotE    = (Xdot * (L2 * cos(qS + qE)) + Ydot * (L2 * sin(qS + qE)))/(L1 * L2 * sin(qE));
qDotS = (Xdot * (L1 * cos(qS) - L2 * cos(qS + qE)) + Ydot * (-L1 * sin(qS) - L2 * sin(qS + qE)))/(L1 * L2 * sin(qE));

end