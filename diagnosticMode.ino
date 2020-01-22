/* This function is used when troubleshooting the robot.
   Enable this mode during the setup loop
   Function by Erick Nunez
   created 4/18/2019
*/

void diagnosticMode(unsigned long totalTime, forceStruct rawF, forceStruct F, jointSpace pres, modelSpace init, modelSpace goal, jointSpace Q, unsigned long goalReturn, unsigned long loopTime) {
  Serial.print(totalTime); Serial.print("\t");

  //Serial.print(rawF.X); Serial.print("\t"); Serial.print(rawF.Y); Serial.print("\t"); //Serial.print(rawF.Z); Serial.print("\t");

  Serial.print(F.X); Serial.print("\t"); Serial.print(F.Y); Serial.print("\t"); //Serial.print(F.Z); Serial.print("\t");

  Serial.print(pres.q1); Serial.print("\t"); Serial.print(pres.q2); Serial.print("\t");

  Serial.print(pres.q1Dot); Serial.print("\t"); Serial.print(pres.q2Dot); Serial.print("\t");

  //Serial.print(init.x, 4); Serial.print("\t"); Serial.print(init.y, 4); Serial.print("\t");

  //Serial.print(init.xDot, 4); Serial.print("\t"); Serial.print(init.yDot, 4); Serial.print("\t");

  Serial.print(goal.x, 4); Serial.print("\t"); Serial.print(goal.y, 4); Serial.print("\t");

  Serial.print(goal.xDot, 4); Serial.print("\t"); Serial.print(goal.yDot, 4); Serial.print("\t");

  Serial.print(Q.q1); Serial.print("\t"); Serial.print(Q.q2); Serial.print("\t");

  Serial.print(Q.q1Dot); Serial.print("\t"); Serial.print(Q.q2Dot); Serial.print("\t");
  
  Serial.print(goalReturn); Serial.print("\t"); Serial.print(loopTime);
  
  Serial.write(13); Serial.write(10); // CR/LF "Carriage Return" and "Linefeed"
}

