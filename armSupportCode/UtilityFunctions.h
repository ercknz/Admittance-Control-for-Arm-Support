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

int32_t * floatToIntArray(float * floatData);

byte * int32ToByteArray(int32_t * int32Data) ;

typedef union {
  float * floatVal;
  byte   byteFloat[12];
} floatToBytes;

#endif // UTILITY_FUNCTIONS_H
