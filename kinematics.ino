/* These functions are used to solve the forward or inverse kinematics of the system in X and Y directions.
   forwardKine() takes the motor counts and outputs the position(SI) for the system.
   inverseKine() takes the position(SI) and outputs the motor counts for the new position.
   
   Created 1/24/2019
   Script by erick nunez
*/



void forwardKine(){
  // motor counts/speed --> forwardKine() --> position/velocity(SI)
  
  presElbowAngVel    = presVelElbow    * RPM_PER_COUNT * (2 * PI / 60);
  presShoulderAngVel = presVelShoulder * RPM_PER_COUNT * (2 * PI / 60);
   
  //xPresPosSI = SHOULDER_ELBOW_LINK * cos(shoulderAngle) + ELBOW_SENSOR_LINK * cos(shoulderAngle+elbowAngle);
  //yPresPosSI = SHOULDER_ELBOW_LINK * sin(shoulderAngle) + ELBOW_SENSOR_LINK * cos(shoulderAngle+elbowAngle);
  xPresVelSI = presShoulderAngVel * (-SHOULDER_ELBOW_LINK * sin(presShoulderAng) - ELBOW_SENSOR_LINK * sin(presShoulderAng + presElbowAng)) + presElbowAngVel * (-ELBOW_SENSOR_LINK * sin(presShoulderAng + presElbowAng));
  yPresVelSI = presShoulderAngVel * ( SHOULDER_ELBOW_LINK * cos(presShoulderAng) + ELBOW_SENSOR_LINK * cos(presShoulderAng + presElbowAng)) + presElbowAngVel * ( ELBOW_SENSOR_LINK * cos(presShoulderAng + presElbowAng));
  
  if (Fx < 0) {
    xPresVelSI = - xPresVelSI;
  }
  if (Fy < 0) {
    yPresVelSI = -yPresVelSI;
  }
    
}

void inverseKine(){
  // position/velocity(SI) --> inverseKine() --> motor counts/speed
  
  goalElbowAng = acos((pow(xGoalPosSI,2) + pow(yGoalPosSI,2) - pow(SHOULDER_ELBOW_LINK,2) - pow(ELBOW_SENSOR_LINK,2))/(2 * SHOULDER_ELBOW_LINK * ELBOW_SENSOR_LINK));
  goalShoulderAng = atan(yGoalPosSI/xGoalPosSI) - atan((ELBOW_SENSOR_LINK * sin(goalElbowAng))/(SHOULDER_ELBOW_LINK + ELBOW_SENSOR_LINK * cos(goalElbowAng)));
  goalElbowAngVel    = (xGoalVelSI * (ELBOW_SENSOR_LINK * cos(goalShoulderAng + goalElbowAng)) + yGoalVelSI * (ELBOW_SENSOR_LINK * sin(goalShoulderAng + goalElbowAng)))/(SHOULDER_ELBOW_LINK * ELBOW_SENSOR_LINK * sin(goalElbowAng));
  goalShoulderAngVel = (xGoalVelSI * (SHOULDER_ELBOW_LINK * cos(goalShoulderAng) - ELBOW_SENSOR_LINK * cos(goalShoulderAng + goalElbowAng)) + yGoalVelSI * (-SHOULDER_ELBOW_LINK * sin(goalShoulderAng) - ELBOW_SENSOR_LINK * sin(goalShoulderAng + goalElbowAng)))/(SHOULDER_ELBOW_LINK * ELBOW_SENSOR_LINK * sin(goalElbowAng));
  
  goalPosElbow    = presPosElbow    + goalElbowAng    * (180/PI) / DEGREES_PER_COUNT;
  goalPosShoulder = presPosShoulder + goalShoulderAng * (180/PI) / DEGREES_PER_COUNT;
  goalVelElbow    = abs(goalElbowAngVel    * (60 / (2 * PI)) / RPM_PER_COUNT);
  goalVelShoulder = abs(goalShoulderAngVel * (60 / (2 * PI)) / RPM_PER_COUNT);
}
