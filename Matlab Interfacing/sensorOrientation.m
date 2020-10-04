function F = sensorOrientation(rawForceX, rawForceY, q1, q2)
% This function is used to reorient the X and Y forces with respect to the ground reference frame.
% This function takes in the X and Y forces as well as the angles of the shoulder and elbow.
% 
% Created 1/24/2019
% Script by erick nunez

%% Rotation Matrix
Ry = @(theta) [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
Rz = @(theta) [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];           
           
%% Finds Global forces           
f = [rawForceX, rawForceY, 0]';
F = Rz(q1)*Rz(q2)*Rz(pi/2)*f;
end