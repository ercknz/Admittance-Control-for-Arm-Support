/* These functions are used to solve the forward or inverse kinematics of the system in X and Y directions.
   forwardKine() takes the motor counts and outputs the position(SI) for the system.
   inverseKine() takes the position(SI) and outputs the motor counts for the new position.
   
   Created 1/24/2019
   Script by erick nunez
*/



void forwardKine(int32_t presVelElbow, int32_t  presVelShoulder, float presShoulderAng, float presElbowAng, float &xPresPosSI, float &yPresPosSI, float &xPresVelSI, float &yPresVelSI){
//void forwardKine(){
  // motor counts/speed --> forwardKine() --> position/velocity(SI)
  
  // Convert motor counts to RPM
  presElbowAngVel    = presVelElbow    * RPM_PER_COUNT * (2.0 * PI / 60.0);
  presShoulderAngVel = presVelShoulder * RPM_PER_COUNT * (2.0 * PI / 60.0);
  
  // Compute the XY positions from angles 
  xPresPosSI = SHOULDER_ELBOW_LINK * cos(presShoulderAng) + ELBOW_SENSOR_LINK * cos(presShoulderAng+presElbowAng);
  yPresPosSI = SHOULDER_ELBOW_LINK * sin(presShoulderAng) + ELBOW_SENSOR_LINK * sin(presShoulderAng+presElbowAng);
  
  // Multiply velocities with Jacobian Matrix to find the XY velocities
  xPresVelSI = presShoulderAngVel * (-SHOULDER_ELBOW_LINK * sin(presShoulderAng) - ELBOW_SENSOR_LINK * sin(presShoulderAng + presElbowAng)) + presElbowAngVel * (-ELBOW_SENSOR_LINK * sin(presShoulderAng + presElbowAng));
  yPresVelSI = presShoulderAngVel * ( SHOULDER_ELBOW_LINK * cos(presShoulderAng) + ELBOW_SENSOR_LINK * cos(presShoulderAng + presElbowAng)) + presElbowAngVel * ( ELBOW_SENSOR_LINK * cos(presShoulderAng + presElbowAng));
}

void inverseKine(float xGoalPosSI, float yGoalPosSI, float xGoalVelSI, float yGoalVelSI, float &goalElbowAng, float &goalShoulderAng, float &goalElbowAngVel, float &goalShoulderAngVel, int32_t &goalPosElbow, int32_t &goalPosShoulder, int32_t &goalVelElbow, int32_t &goalVelShoulder){
//void inverseKine(){
  // position/velocity(SI) --> inverseKine() --> motor counts/speed
 
  // Solving for joint angles
  goalElbowAng = acos((pow(xGoalPosSI,2) + pow(yGoalPosSI,2) - pow(SHOULDER_ELBOW_LINK,2) - pow(ELBOW_SENSOR_LINK,2))/(2.0 * SHOULDER_ELBOW_LINK * ELBOW_SENSOR_LINK));
  if (yGoalPosSI < 0){
    goalShoulderAng = atan2(yGoalPosSI,xGoalPosSI) - atan2((ELBOW_SENSOR_LINK * sin(goalElbowAng)),(SHOULDER_ELBOW_LINK + ELBOW_SENSOR_LINK * cos(goalElbowAng))) + 2.0*PI;
  } else {
    goalShoulderAng = atan2(yGoalPosSI,xGoalPosSI) - atan2((ELBOW_SENSOR_LINK * sin(goalElbowAng)),(SHOULDER_ELBOW_LINK + ELBOW_SENSOR_LINK * cos(goalElbowAng)));
  }
  goalElbowAngVel    = (xGoalVelSI * (ELBOW_SENSOR_LINK * cos(goalShoulderAng + goalElbowAng)) + yGoalVelSI * (ELBOW_SENSOR_LINK * sin(goalShoulderAng + goalElbowAng)))/(SHOULDER_ELBOW_LINK * ELBOW_SENSOR_LINK * sin(goalElbowAng));
  goalShoulderAngVel = (xGoalVelSI * (SHOULDER_ELBOW_LINK * cos(goalShoulderAng) - ELBOW_SENSOR_LINK * cos(goalShoulderAng + goalElbowAng)) + yGoalVelSI * (-SHOULDER_ELBOW_LINK * sin(goalShoulderAng) - ELBOW_SENSOR_LINK * sin(goalShoulderAng + goalElbowAng)))/(SHOULDER_ELBOW_LINK * ELBOW_SENSOR_LINK * sin(goalElbowAng));
  
  // Finding goal motor counts from join angles. 
  goalPosElbow    = ELBOW_MAX_POS - goalElbowAng * (180.0/PI) / DEGREES_PER_COUNT;
  goalPosShoulder = goalShoulderAng * (180.0/PI) / DEGREES_PER_COUNT;
  goalVelElbow    = abs(goalElbowAngVel    * (60.0 / (2.0 * PI)) / RPM_PER_COUNT);
  goalVelShoulder = abs(goalShoulderAngVel * (60.0 / (2.0 * PI)) / RPM_PER_COUNT);
}
