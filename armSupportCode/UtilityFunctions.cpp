/* Utility functions used with the arm support robot.

   Packet Reference:
   [   1,  2,  3,  4,  5,  6,  7,     8,     9,    10,11,12,13,14,15,16, 17, 18, 19,    20,    21,    22,     23, 24]
   [time, Fx, Fy, Fz, q1, q2, q4, q1Dot, q2Dot, q4Dot, x, y, z, u, v, w, Q1, Q2, Q4, Q1Dot, Q2Dot, Q4Dot, return, dT]

   Created 10/28/2020
   by Erick Nunez
*/

#include "AdmittanceModel.h"
#include "ForceSensor.h"
#include "RobotControl.h"
#include "UtilityFunctions.h"

/******************** Bytes to Counts Converter  ***********************************************************************/
int16_t bytesToCounts(byte hByte, byte lByte) {
  int16_t decimal = hByte * 256 + lByte;
  return decimal;
}

/******************** Streaming Function  ***********************************************************************/
void loggingFunc(unsigned long &totalTime, ForceSensor *Sensor, AdmittanceModel *Model, RobotControl *Robot, unsigned long &loopTime) {
  float RawF[3]       = Sensor -> GetRawF();
  float GlobalF[3]    = Sensor -> GetGlobalF();
  float xyzGoal[3]    = Model -> GetGoalPos();
  float xyzDotGoal[3] = Model -> GetGoalVel();
  float PresQCts[3]   = Robot -> GetPresQCts();
  float PresQDotCts[3]= Robot -> GetPresQDotCts();
  float PresQ[3]      = Robot -> GetPresQ();
  float PresQDot[3]   = Robot -> GetPresQDot();
  float PresPos[3]    = Robot -> GetPresPos();
  float PresVel[3]    = Robot -> GetPresVel();
  float GoalQCts[3]   = Robot -> GetGoalQCts();
  float GoalQDotCts[3]= Robot -> GetGoalQDotCts();
  float GoalQ[3]      = Robot -> GetGoalQ();
  float GoalQDot[3]   = Robot -> GetGoalQDot();
  
  Serial.print(totalTime); Serial.print("\t");

  Serial.print(RawF[0]); Serial.print("\t"); Serial.print(RawF[1]); Serial.print("\t"); Serial.print(RawF[2]); Serial.print("\t");

  Serial.print(PresQCts[0]); Serial.print("\t"); Serial.print(PresQCts[1]); Serial.print("\t"); Serial.print(PresQCts[2]); Serial.print("\t");

  Serial.print(PresQ[0]); Serial.print("\t"); Serial.print(PresQ[1]); Serial.print("\t"); Serial.print(PresQ[2]); Serial.print("\t");

  Serial.print(PresPos[0]); Serial.print("\t"); Serial.print(PresPos[1]); Serial.print("\t"); Serial.print(PresPos[2]); Serial.print("\t");
  
  Serial.print(loopTime);
  
  Serial.write(13); Serial.write(10); // CR/LF "Carriage Return" and "Linefeed"
}
