/* Utility functions used with the arm support robot.

   Packet Reference:
   [   1,  2,  3,  4,  5,  6,  7,     8,     9,    10,11,12,13,14,15,16, 17, 18, 19,    20,    21,    22,     23, 24]
   [time, Fx, Fy, Fz, q1, q2, q4, q1Dot, q2Dot, q4Dot, x, y, z, u, v, w, Q1, Q2, Q4, Q1Dot, Q2Dot, Q4Dot, dT]

   Created 10/28/2020
   by Erick Nunez
*/

#include "ForceSensor.h"
#include "AdmittanceModel.h"
#include "RobotControl.h"
#include "UtilityFunctions.h"

/******************** Bytes to Counts Converter  ***********************************************************************/
int16_t bytesToCounts(byte hByte, byte lByte) {
  int16_t decimal = hByte * 256 + lByte;
  return decimal;
}

/******************** Streaming Function  ***********************************************************************/
void loggingFunc(unsigned long &totalTime, ForceSensor &Sensor, AdmittanceModel &Model, RobotControl &Robot, unsigned long &loopTime) {
  float *   RawF          = Sensor.GetRawF();
  float *   GlobalF       = Sensor.GetGlobalF();
  float *   xyzGoal       = Model.GetGoalPos();
  float *   xyzDotGoal    = Model.GetGoalVel();
  float *   xyzBotGoal    = Robot.GetGoalPos();
  float *   xyzDotBotGoal = Robot.GetGoalVel();
  int32_t * PresQCts      = Robot.GetPresQCts();
  int32_t * PresQDotCts   = Robot.GetPresQDotCts();
  float *   PresQ         = Robot.GetPresQ();
  float *   PresQDot      = Robot.GetPresQDot();
  float *   PresPos       = Robot.GetPresPos();
  float *   PresVel       = Robot.GetPresVel();
  int32_t * GoalQCts      = Robot.GetGoalQCts();
  int32_t * GoalQDotCts   = Robot.GetGoalQDotCts();
  float *   GoalQ         = Robot.GetGoalQ();
  float *   GoalQDot      = Robot.GetGoalQDot();
  
  Serial.print(totalTime); Serial.print("\t");

  Serial.print(RawF[0]); Serial.print("\t"); Serial.print(RawF[1]); Serial.print("\t"); Serial.print(RawF[2]); Serial.print("\t");

  Serial.print(GlobalF[0]); Serial.print("\t"); Serial.print(GlobalF[1]); Serial.print("\t"); Serial.print(GlobalF[2]); Serial.print("\t");

  Serial.print(PresQ[0]); Serial.print("\t"); Serial.print(PresQ[1]); Serial.print("\t"); Serial.print(PresQ[2]); Serial.print("\t");

  //Serial.print(PresQDot[0]); Serial.print("\t"); Serial.print(PresQDot[1]); Serial.print("\t"); Serial.print(PresQDot[2]); Serial.print("\t");
  
  Serial.print(xyzBotGoal[0]); Serial.print("\t"); Serial.print(xyzBotGoal[1]); Serial.print("\t"); Serial.print(xyzBotGoal[2]); Serial.print("\t");

  Serial.print(xyzDotBotGoal[0]); Serial.print("\t"); Serial.print(xyzDotBotGoal[1]); Serial.print("\t"); Serial.print(xyzDotBotGoal[2]); Serial.print("\t");

  Serial.print(GoalQ[0]); Serial.print("\t"); Serial.print(GoalQ[1]); Serial.print("\t"); Serial.print(GoalQ[2]); Serial.print("\t");

  //Serial.print(GoalQDot[0]); Serial.print("\t"); Serial.print(GoalQDot[1]); Serial.print("\t"); Serial.print(GoalQDot[2]); Serial.print("\t");
  
  Serial.print(loopTime);
  
  Serial.write(13); Serial.write(10); // CR/LF "Carriage Return" and "Linefeed"
}
