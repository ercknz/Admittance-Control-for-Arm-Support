/* These functions are used to solve the forward or inverse kinematics of the system in X and Y directions.
   forwardKine() takes the motor counts and outputs the position(SI) for the system.
   inverseKine() takes the position(SI) and outputs the motor counts for the new position.
   
   Created 1/24/2019
   Script by erick nunez
*/



void forwardKine(double xForce, double yForce, 
                 int32 presVelElbow, int32 presVelShoulder, 
                 float &xPresPosSI,  float &xPresVelSI,  float &yPresPosSI,     float &yPresVelSI){
  // motor counts/speed --> forwardKine() --> position/velocity(SI)
  
  //elbowAngle     = presPosElbow    * DEGREES_PER_COUNT * (PI/180);
  //shoulderAngle  = presPosShoulder * DEGREES_PER_COUNT * (PI/180);
  elbowAngVel    = presVelElbow    * RPM_PER_COUNT * (2 * PI / 60);
  shoulderAngVel = presVelShoulder * RPM_PER_COUNT * (2 * PI / 60);
   
  //xPresPosSI = SHOULDER_ELBOW_LINK * cos(shoulderAngle) + ELBOW_SENSOR_LINK * cos(shoulderAngle+elbowAngle);
  //yPresPosSI = SHOULDER_ELBOW_LINK * sin(shoulderAngle) + ELBOW_SENSOR_LINK * cos(shoulderAngle+elbowAngle);
  xPresVelSI = shoulderAngVel * (-SHOULDER_ELBOW_LINK * sin(shoulderAngle) - ELBOW_SENSOR_LINK * sin(shoulderAngle+elbowAngle)) + elbowAngVel * (-ELBOW_SENSOR_LINK * sin(shoulderAngle+elbowAngle));
  yPresVelSI = shoulderAngVel * ( SHOULDER_ELBOW_LINK * cos(shoulderAngle) + ELBOW_SENSOR_LINK * cos(shoulderAngle+elbowAngle)) + elbowAngVel * ( ELBOW_SENSOR_LINK * cos(shoulderAngle+elbowAngle));
  
  if (xForce < 0) {
    xPresVelSI = - xPresVelSI;
  }
  if (yForce < 0) {
    yPresVelSI = -yPresVelSI;
  }
    
}

void inverseKine(float xGoalPosSI,  float xGoalVelSI,  float yGoalPosSI,     float yGoalVelSI, 
                 int32 presPosElbow, int32 presVelElbow,
                 int &goalPosElbow, int &goalVelElbow, int &goalPosShoulder, int &goalVelShoulder){
  // position/velocity(SI) --> inverseKine() --> motor counts/speed
  
  elbowAngle = acos((pow(xGoalPosSI,2) + pow(yGoalPosSI,2) - pow(SHOULDER_ELBOW_LINK,2) - pow(ELBOW_SENSOR_LINK,2))/(2 * SHOULDER_ELBOW_LINK * ELBOW_SENSOR_LINK));
  shoulderAngle = atan(yGoalPosSI/xGoalPosSI) - atan((ELBOW_SENSOR_LINK * sin(elbowAngle))/(SHOULDER_ELBOW_LINK + ELBOW_SENSOR_LINK * cos(elbowAngle)));
  elbowAngVel    = (xGoalVelSI * (ELBOW_SENSOR_LINK * cos(shoulderAngle+elbowAngle)) + yGoalVelSI * (ELBOW_SENSOR_LINK * sin(shoulderAngle+elbowAngle)))/(SHOULDER_ELBOW_LINK * ELBOW_SENSOR_LINK * sin(elbowAngle));
  shoulderAngVel = (xGoalVelSI * (SHOULDER_ELBOW_LINK * cos(shoulderAngle) - ELBOW_SENSOR_LINK * cos(shoulderAngle+elbowAngle)) + yGoalVelSI * (-SHOULDER_ELBOW_LINK * sin(shoulderAngle) - ELBOW_SENSOR_LINK * sin(shoulderAngle+elbowAngle)))/(SHOULDER_ELBOW_LINK * ELBOW_SENSOR_LINK * sin(elbowAngle));
  
  goalPosElbow    = presPosElbow    + elbowAngle    * (180/PI) / DEGREES_PER_COUNT;
  goalPosShoulder = presPosShoulder + shoulderAngle * (180/PI) / DEGREES_PER_COUNT;
  goalVelElbow    = abs(elbowAngVel    * (60 / (2 * PI)) / RPM_PER_COUNT);
  goalVelShoulder = abs(shoulderAngVel * (60 / (2 * PI)) / RPM_PER_COUNT);
}
