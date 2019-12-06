function [x,y,u,v] = armSupportFKine(qE, qS, qDotE, qDotS)
%% Arm Support Forward Kinematics
% This function is used to calculate the position and velocity of the end
% effector of the robot in the workpace given the joint angles and
% velocities.
%
% Script by erick nunez
%%
presElbowAngVel    = presVelElbow    * RPM_PER_COUNT * (2.0 * PI / 60.0);
presShoulderAngVel = presVelShoulder * RPM_PER_COUNT * (2.0 * PI / 60.0);


xPresPosSI = SHOULDER_ELBOW_LINK * cos(presShoulderAng) + ELBOW_SENSOR_LINK * cos(presShoulderAng+presElbowAng);
yPresPosSI = SHOULDER_ELBOW_LINK * sin(presShoulderAng) + ELBOW_SENSOR_LINK * sin(presShoulderAng+presElbowAng);


xPresVelSI = presShoulderAngVel * (-SHOULDER_ELBOW_LINK * sin(presShoulderAng) - ELBOW_SENSOR_LINK * sin(presShoulderAng + presElbowAng)) + presElbowAngVel * (-ELBOW_SENSOR_LINK * sin(presShoulderAng + presElbowAng));
yPresVelSI = presShoulderAngVel * ( SHOULDER_ELBOW_LINK * cos(presShoulderAng) + ELBOW_SENSOR_LINK * cos(presShoulderAng + presElbowAng)) + presElbowAngVel * ( ELBOW_SENSOR_LINK * cos(presShoulderAng + presElbowAng));

end