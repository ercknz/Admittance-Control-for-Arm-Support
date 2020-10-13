/* This function is used when troubleshooting the robot.
   Enable this mode during the setup loop
   Function by Erick Nunez
   created 4/18/2019
   
   Packet Reference:
   [   1,  2,  3,  4,  5,  6,  7,     8,     9,    10,11,12,13,14,15,16, 17, 18, 19,    20,    21,    22,     23, 24]
   [time, Fx, Fy, Fz, q1, q2, q4, q1Dot, q2Dot, q4Dot, x, y, z, u, v, w, Q1, Q2, Q4, Q1Dot, Q2Dot, Q4Dot, return, dT]
*/

void loggingFunc(unsigned long totalTime, forceStruct rawF, forceStruct F, jointSpace pres, modelSpace curr, modelSpace init, modelSpace goal, jointSpace Q, unsigned long goalReturn, unsigned long loopTime) {
  Serial.print(totalTime); Serial.print("\t");

  //Serial.print(rawF.X); Serial.print("\t"); Serial.print(rawF.Y); Serial.print("\t"); Serial.print(rawF.Z); Serial.print("\t");

  Serial.print(F.X); Serial.print("\t"); Serial.print(F.Y); Serial.print("\t"); Serial.print(F.Z); Serial.print("\t");

  //Serial.print(pres.q1Cts); Serial.print("\t"); Serial.print(pres.q2Cts); Serial.print("\t"); Serial.print(pres.q4Cts); Serial.print("\t");

  //Serial.print(pres.q1); Serial.print("\t"); Serial.print(pres.q2); Serial.print("\t"); Serial.print(pres.q4); Serial.print("\t");

  Serial.print(init.x); Serial.print("\t"); Serial.print(init.y); Serial.print("\t"); Serial.print(init.z); Serial.print("\t");

  //Serial.print(pres.q1DotCts); Serial.print("\t"); Serial.print(pres.q2DotCts); Serial.print("\t"); Serial.print(pres.q4DotCts); Serial.print("\t");

  Serial.print(goal.x); Serial.print("\t"); Serial.print(goal.y); Serial.print("\t"); Serial.print(goal.z); Serial.print("\t");

  //Serial.print(goal.xDot); Serial.print("\t"); Serial.print(goal.yDot); Serial.print("\t"); Serial.print(goal.zDot); Serial.print("\t");

  //Serial.print(Q.q1); Serial.print("\t"); Serial.print(Q.q2); Serial.print("\t"); Serial.print(Q.q4); Serial.print("\t");

  //Serial.print(Q.q1Dot); Serial.print("\t"); Serial.print(Q.q2Dot); Serial.print("\t"); Serial.print(Q.q4Dot); Serial.print("\t");
  
  Serial.print(goalReturn); Serial.print("\t"); Serial.print(loopTime);
  
  Serial.write(13); Serial.write(10); // CR/LF "Carriage Return" and "Linefeed"
}

