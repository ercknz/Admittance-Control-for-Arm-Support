function [X,Y,Xdot,Ydot] = armSupportFKine(qS, qE, qDotS, qDotE)
%% Arm Support Forward Kinematics
% This function is used to calculate the position and velocity of the end
% effector of the robot in the workpace given the joint angles and
% velocities.
%
% Script by erick nunez

%% constants
L1 = 0.510;
L2 = 0.505;

%% Calculate
X = L1 * cos(qS) + L2 * cos(qS+qE);
Y = L1 * sin(qS) + L2 * sin(qS+qE);

Xdot = qDotS * (-L1 * sin(qS) - L2 * sin(qS + qE)) + qDotE * (-L2 * sin(qS + qE));
Ydot = qDotS * ( L1 * cos(qS) + L2 * cos(qS + qE)) + qDotE * ( L2 * cos(qS + qE));

end