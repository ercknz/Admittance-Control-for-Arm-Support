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

/******************** Float to Int Array Function  ***********************************************************************/
int32_t * floatToIntArray(float * floatData) {
  static int32_t intOut[3];
  for (int16_t i = 0; i < 3; i++) {
    intOut[i] = (int32_t)(floatData[i] * 1000.0);
  }
  return intOut;
}

/******************** Int32 to Byte Array Function  ***********************************************************************/
byte * int32ToByteArray(int32_t * int32Data) {
  static byte bytesOut[12];
  for (int16_t i = 0; i < 3; i++) {
    bytesOut[4 * i]     = DXL_LOBYTE(DXL_LOWORD(int32Data[i]));
    bytesOut[4 * i + 1] = DXL_HIBYTE(DXL_LOWORD(int32Data[i]));
    bytesOut[4 * i + 2] = DXL_LOBYTE(DXL_HIWORD(int32Data[i]));
    bytesOut[4 * i + 3] = DXL_HIBYTE(DXL_HIWORD(int32Data[i]));
  }
  return bytesOut;
}

