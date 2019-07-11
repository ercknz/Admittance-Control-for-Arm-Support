/* This function is used when troubleshooting the robot.
 * Enable this mode during the setup loop
 * Function by Erick Nunez
 * created 4/18/2019
 */

void diagnosticMode(){
  //Serial.print(encoderCounter); Serial.print("\t");
  //Serial.print(inputPID); Serial.print("\t");
  //Serial.print(setPointPID); Serial.print("\t");
  //Serial.print(outputPID); Serial.print("\t");

  Serial.print(xRaw); Serial.print("\t"); Serial.print(yRaw); Serial.print("\t"); Serial.print(zRaw); Serial.print("\t"); Serial.print("||\t");
  
  Serial.print(FxRaw); Serial.print("\t"); Serial.print(FyRaw); Serial.print("\t"); Serial.print(FzRaw); Serial.print("\t"); Serial.print("||\t");

  Serial.print(Fx); Serial.print("\t"); Serial.print(Fy); Serial.print("\t"); Serial.print(Fz); Serial.print("\t"); Serial.print("||\t");

  Serial.print(presPosElbow); Serial.print("\t"); Serial.print(presPosShoulder); Serial.print("\t"); Serial.print("||\t");

  Serial.print(presElbowAng); Serial.print("\t"); Serial.print(presShoulderAng); Serial.print("\t"); Serial.print("||\t");

  //Serial.print(presVelElbow); Serial.print("\t"); 
  //Serial.print(presVelShoulder); Serial.print("\t");

  //Serial.print(xPresPosSI); Serial.print("\t");
  //Serial.print(xPresVelSI); Serial.print("\t");

  //Serial.print(yPresPosSI); Serial.print("\t");
  //Serial.print(yPresVelSI); Serial.print("\t"); 

  //Serial.print(xGoalPosSI); Serial.print("\t");
  //Serial.print(xGoalVelSI); Serial.print("\t");

  //Serial.print(yGoalPosSI); Serial.print("\t");
  //Serial.print(yGoalVelSI); Serial.print("\t");

  //Serial.print(goalElbowAng); Serial.print("\t"); Serial.print(goalShoulderAng); Serial.print("\t");

  //Serial.print(goalElbowAngVel); Serial.print("\t");
  //Serial.print(goalShoulderAngVel); Serial.print("\t");

  //Serial.print(goalVelElbow); Serial.print("\t"); 
  //Serial.print(goalVelShoulder); Serial.print("\t");

  //Serial.print(goalPosElbow); Serial.print("\t"); Serial.print(goalPosShoulder); Serial.print("\t");

  Serial.print(postTime-preTime); Serial.print("\t"); 
  Serial.print(goalReturn); Serial.print("\t");  
  Serial.print("\n");
}

