/* Utility functions used with the arm support robot.

   Packet Reference:
   [   1,  2,  3,  4,  5,  6,  7,     8,     9,    10,11,12,13,14,15,16, 17, 18, 19,    20,    21,    22,     23, 24]
   [time, Fx, Fy, Fz, q1, q2, q4, q1Dot, q2Dot, q4Dot, x, y, z, u, v, w, Q1, Q2, Q4, Q1Dot, Q2Dot, Q4Dot, return, dT]

   Created 10/28/2020
   by Erick Nunez
*/

#include "UtilityFunctions.h"
#include "AdmittanceModel.h"
#include "ForceSensor.h"
#include "RobotControl.h"

/******************** Bytes to Counts Converter  ***********************************************************************/
int16_t bytesToCounts(byte hByte, byte lByte) {
  int16_t decimal = hByte * 256 + lByte;
  return decimal;
}

/******************** Streaming Function  ***********************************************************************/
void loggingFunc(unsigned long &totalTime, ForceSensor *Sensor, AdmittanceModel *Model, RobotControl *Robot, unsigned long &loopTime) {
  float F[3] = ForceSensor.Get
  
  Serial.print(totalTime); Serial.print("\t");

  Serial.print(F.X); Serial.print("\t"); Serial.print(F.Y); Serial.print("\t"); Serial.print(F.Z); Serial.print("\t");

  Serial.print(pres.q1); Serial.print("\t"); Serial.print(pres.q2); Serial.print("\t"); Serial.print(pres.q4); Serial.print("\t");

  Serial.print(init.x); Serial.print("\t"); Serial.print(init.y); Serial.print("\t"); Serial.print(init.z); Serial.print("\t");

  Serial.print(goal.x); Serial.print("\t"); Serial.print(goal.y); Serial.print("\t"); Serial.print(goal.z); Serial.print("\t");

  Serial.print(Q.q1); Serial.print("\t"); Serial.print(Q.q2); Serial.print("\t"); Serial.print(Q.q4); Serial.print("\t");
  
  Serial.print(loopTime);
  
  Serial.write(13); Serial.write(10); // CR/LF "Carriage Return" and "Linefeed"
}
