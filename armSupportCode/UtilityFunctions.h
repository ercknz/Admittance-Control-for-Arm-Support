/* Utility functions used with the arm support robot.

   Created 10/28/2020
   by Erick Nunez
*/

#ifndef UTILITY_FUNCTIONS_H
#define UTILITY_FUNCTIONS_H

#include "ForceSensor.h"
#include "AdmittanceModel.h"
#include "RobotControl.h"

int16_t bytesToCounts(byte hByte, byte lByte);

void loggingFunc(unsigned long &totalTime, ForceSensor &OptoSensor, AdmittanceModel &Model, RobotControl &Robot, unsigned long &loopTime);

typedef union {
  float floatVal;
  byte  byteFloat[4];
} floatToBytes;

#endif // UTILITY_FUNCTIONS_H
