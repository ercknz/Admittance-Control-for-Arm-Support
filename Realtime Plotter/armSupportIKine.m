function [qE, qS, qDot] = armSupportIKine(x,y,u,v)
%% Arm Support Inverse Kinematics
% This function is used to calculate the joint angles and angular
% velocities of the robot given the position and velocity of the object in
% the workspace.
%
% Script by erick nunez

goalElbowAng = acos((pow(xGoalPosSI,2) + pow(yGoalPosSI,2) - pow(SHOULDER_ELBOW_LINK,2) - pow(ELBOW_SENSOR_LINK,2))/(2.0 * SHOULDER_ELBOW_LINK * ELBOW_SENSOR_LINK));
if yGoalPosSI < 0
    goalShoulderAng = atan2(yGoalPosSI,xGoalPosSI) - atan2((ELBOW_SENSOR_LINK * sin(goalElbowAng)),(SHOULDER_ELBOW_LINK + ELBOW_SENSOR_LINK * cos(goalElbowAng))) + 2.0*PI;
else
    goalShoulderAng = atan2(yGoalPosSI,xGoalPosSI) - atan2((ELBOW_SENSOR_LINK * sin(goalElbowAng)),(SHOULDER_ELBOW_LINK + ELBOW_SENSOR_LINK * cos(goalElbowAng)));
end
goalElbowAngVel    = (xGoalVelSI * (ELBOW_SENSOR_LINK * cos(goalShoulderAng + goalElbowAng)) + yGoalVelSI * (ELBOW_SENSOR_LINK * sin(goalShoulderAng + goalElbowAng)))/(SHOULDER_ELBOW_LINK * ELBOW_SENSOR_LINK * sin(goalElbowAng));
goalShoulderAngVel = (xGoalVelSI * (SHOULDER_ELBOW_LINK * cos(goalShoulderAng) - ELBOW_SENSOR_LINK * cos(goalShoulderAng + goalElbowAng)) + yGoalVelSI * (-SHOULDER_ELBOW_LINK * sin(goalShoulderAng) - ELBOW_SENSOR_LINK * sin(goalShoulderAng + goalElbowAng)))/(SHOULDER_ELBOW_LINK * ELBOW_SENSOR_LINK * sin(goalElbowAng));

goalPosElbow    = ELBOW_MIN_POS + goalElbowAng * (180.0/PI) / DEGREES_PER_COUNT;
goalPosShoulder = goalShoulderAng * (180.0/PI) / DEGREES_PER_COUNT;
goalVelElbow    = abs(goalElbowAngVel    * (60.0 / (2.0 * PI)) / RPM_PER_COUNT);
goalVelShoulder = abs(goalShoulderAngVel * (60.0 / (2.0 * PI)) / RPM_PER_COUNT);

end