/* These functions are used to solve the forward or inverse kinematics of the system in X and Y directions.
   forwardKine() takes the motor counts and outputs the position(SI) for the system.
   inverseKine() takes the position(SI) and outputs the motor counts for the new position.
   
   Created 1/24/2019
   Script by erick nunez
*/



void forwardKine(int32_t presVelElbow, int32_t  presVelShoulder, float presShoulderAng, float presElbowAng, float &xPresPosSI, float &yPresPosSI, float &xPresVelSI, float &yPresVelSI){
//void forwardKine(){
  // motor counts/speed --> forwardKine() --> position/velocity(SI)
  
  presElbowAngVel    = presVelElbow    * RPM_PER_COUNT * (2 * PI / 60);
  presShoulderAngVel = presVelShoulder * RPM_PER_COUNT * (2 * PI / 60);
   
  xPresPosSI = SHOULDER_ELBOW_LINK * cos(presShoulderAng) + ELBOW_SENSOR_LINK * cos(presShoulderAng+presElbowAng);
  yPresPosSI = SHOULDER_ELBOW_LINK * sin(presShoulderAng) + ELBOW_SENSOR_LINK * sin(presShoulderAng+presElbowAng);
  xPresVelSI = presShoulderAngVel * (-SHOULDER_ELBOW_LINK * sin(presShoulderAng) - ELBOW_SENSOR_LINK * sin(presShoulderAng + presElbowAng)) + presElbowAngVel * (-ELBOW_SENSOR_LINK * sin(presShoulderAng + presElbowAng));
  yPresVelSI = presShoulderAngVel * ( SHOULDER_ELBOW_LINK * cos(presShoulderAng) + ELBOW_SENSOR_LINK * cos(presShoulderAng + presElbowAng)) + presElbowAngVel * ( ELBOW_SENSOR_LINK * cos(presShoulderAng + presElbowAng));
  
  if (Fx < 0) {
    xPresVelSI = - xPresVelSI;
  }
  if (Fy < 0) {
    yPresVelSI = -yPresVelSI;
  }
    
}

void inverseKine(float xGoalPosSI, float yGoalPosSI, float xGoalVelSI, float yGoalVelSI, float &goalElbowAng, float &goalShoulderAng, float &goalElbowAngVel, float &goalShoulderAngVel, int32_t &goalPosElbow, int32_t &goalPosShoulder, int32_t &goalVelElbow, int32_t &goalVelShoulder){
//void inverseKine(){
  // position/velocity(SI) --> inverseKine() --> motor counts/speed
 
  // Goals in radians for troubleshooting.
  goalElbowAng = acos((pow(xGoalPosSI,2) + pow(yGoalPosSI,2) - pow(SHOULDER_ELBOW_LINK,2) - pow(ELBOW_SENSOR_LINK,2))/(2 * SHOULDER_ELBOW_LINK * ELBOW_SENSOR_LINK));
  if (xGoalPosSI < 0){
    goalShoulderAng = atan(yGoalPosSI/xGoalPosSI) - atan((ELBOW_SENSOR_LINK * sin(goalElbowAng))/(SHOULDER_ELBOW_LINK + ELBOW_SENSOR_LINK * cos(goalElbowAng))) + PI;
  } else {
    goalShoulderAng = atan(yGoalPosSI/xGoalPosSI) - atan((ELBOW_SENSOR_LINK * sin(goalElbowAng))/(SHOULDER_ELBOW_LINK + ELBOW_SENSOR_LINK * cos(goalElbowAng)));
  }
  goalElbowAngVel    = (xGoalVelSI * (ELBOW_SENSOR_LINK * cos(goalShoulderAng + goalElbowAng)) + yGoalVelSI * (ELBOW_SENSOR_LINK * sin(goalShoulderAng + goalElbowAng)))/(SHOULDER_ELBOW_LINK * ELBOW_SENSOR_LINK * sin(goalElbowAng));
  goalShoulderAngVel = (xGoalVelSI * (SHOULDER_ELBOW_LINK * cos(goalShoulderAng) - ELBOW_SENSOR_LINK * cos(goalShoulderAng + goalElbowAng)) + yGoalVelSI * (-SHOULDER_ELBOW_LINK * sin(goalShoulderAng) - ELBOW_SENSOR_LINK * sin(goalShoulderAng + goalElbowAng)))/(SHOULDER_ELBOW_LINK * ELBOW_SENSOR_LINK * sin(goalElbowAng));
  
  // Goals in counts for motor communication. 
  goalPosElbow    = ELBOW_MAX_POS - goalElbowAng * (180/PI) / DEGREES_PER_COUNT;
  goalPosShoulder = goalShoulderAng * (180/PI) / DEGREES_PER_COUNT;
  goalVelElbow    = abs(goalElbowAngVel    * (60 / (2 * PI)) / RPM_PER_COUNT);
  goalVelShoulder = abs(goalShoulderAngVel * (60 / (2 * PI)) / RPM_PER_COUNT);

//  // FIX ME !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//  if (goalShoulderAng < 0) {
//    goalPosShoulder = 4000 + goalShoulderAng * (180/PI) / DEGREES_PER_COUNT;
//  } else {
//    
//  }
}
