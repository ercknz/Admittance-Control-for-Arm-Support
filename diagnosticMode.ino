/* This function is used when troubleshooting the robot.
 * Enable this mode during the setup loop
 * Function by Erick Nunez
 * created 4/18/2019
 */

void diagnosticMode(){
  //Serial.print(encoderCounter); Serial.print("\t"); Serial.print("||\t");
  //Serial.print(inputPID); Serial.print("\t"); Serial.print("||\t");
  //Serial.print(setPointPID); Serial.print("\t"); Serial.print("||\t");
  //Serial.print(outputPID); Serial.print("\t"); Serial.print("||\t");

  //Serial.print(xRaw); Serial.print("\t"); Serial.print(yRaw); Serial.print("\t"); Serial.print(zRaw); Serial.print("\t"); Serial.print("||\t");
  
  //Serial.print(FxRaw); Serial.print("\t"); Serial.print(FyRaw); Serial.print("\t"); Serial.print(FzRaw); Serial.print("\t"); Serial.print("||\t");

  Serial.print(Fx); Serial.print("\t"); Serial.print(Fy); Serial.print("\t"); Serial.print(Fz); Serial.print("\t"); Serial.print("||\t");

  //Serial.print(presPosElbow); Serial.print("\t"); Serial.print(presPosShoulder); Serial.print("\t"); Serial.print("||\t");

  //Serial.print(presVelElbow); Serial.print("\t"); Serial.print(presVelShoulder); Serial.print("\t"); Serial.print("||\t");

  //Serial.print(presElbowAng); Serial.print("\t"); Serial.print(presShoulderAng); Serial.print("\t"); Serial.print("||\t");

  //Serial.print(presElbowAngVel); Serial.print("\t"); Serial.print(presShoulderAngVel); Serial.print("\t"); Serial.print("||\t");

  //Serial.print(xPresPosSI,4); Serial.print("\t"); Serial.print(yPresPosSI,4); Serial.print("\t"); Serial.print("||\t");

  //Serial.print(xPresVelSI,4); Serial.print("\t"); Serial.print(yPresVelSI,4); Serial.print("\t"); Serial.print("||\t");

  //Serial.print(xGoalPosSI,4); Serial.print("\t"); Serial.print(yGoalPosSI,4); Serial.print("\t"); Serial.print("||\t");

  //Serial.print(xGoalVelSI,4); Serial.print("\t"); Serial.print(yGoalVelSI,4); Serial.print("\t"); Serial.print("||\t");

  //Serial.print(goalElbowAng); Serial.print("\t"); Serial.print(goalShoulderAng); Serial.print("\t"); Serial.print("||\t");

  //Serial.print(goalElbowAngVel); Serial.print("\t"); Serial.print(goalShoulderAngVel); Serial.print("\t"); Serial.print("||\t");

  //Serial.print(goalPosElbow); Serial.print("\t"); Serial.print(goalPosShoulder); Serial.print("\t"); Serial.print("||\t");

  //Serial.print(goalVelElbow); Serial.print("\t"); Serial.print(goalVelShoulder); Serial.print("\t"); Serial.print("||\t");

  //Serial.print(currentTime-previousTime); Serial.print("\t"); 
  Serial.print(goalReturn); Serial.print("\t");  
  Serial.print("\n");
}

