/* Utility functions used with the arm support robot.

   Packet Reference:
   [   1,  2,  3,  4,  5,  6,  7,     8,     9,    10,11,12,13,14,15,16, 17, 18, 19,    20,    21,    22,     23, 24]
   [time, Fx, Fy, Fz, q1, q2, q4, q1Dot, q2Dot, q4Dot, x, y, z, u, v, w, Q1, Q2, Q4, Q1Dot, Q2Dot, Q4Dot, dT]

   Created 10/28/2020
   by Erick Nunez
*/

#include <DynamixelSDK.h>
#include "UtilityFunctions.h"

/* Bytes to Counts Converter  *************************************************/
int16_t bytesToCounts(byte hByte, byte lByte) {
  int16_t decimal = hByte * 256 + lByte;
  return decimal;
}

/* Float Array to Byte Array Function  ****************************************/
byte * floatArrayToBytes(float * floatValues) {
  int32_t int32Data[3];
  for (int16_t i = 0; i < 3; i++) {
    int32Data[i] = (int32_t)(floatValues[i] * 10000.0);
  }
  byte * bytesOut = int32ArrayToBytes(int32Data);
  return bytesOut;
}

/* Single Float to Byte Array Function  ***************************************/
byte * floatToBytes(float floatValue){
  int32_t int32Data = (int32_t)(floatValue * 10000.0);
  static byte bytesOut[4];
  bytesOut[0] = DXL_LOBYTE(DXL_LOWORD(int32Data));
  bytesOut[1] = DXL_HIBYTE(DXL_LOWORD(int32Data));
  bytesOut[2] = DXL_LOBYTE(DXL_HIWORD(int32Data));
  bytesOut[3] = DXL_HIBYTE(DXL_HIWORD(int32Data));
  return bytesOut;
}

/* Int32 Array to Byte Array Function  ****************************************/
byte * int32ArrayToBytes(int32_t * int32Values) {
  static byte bytesOut[12];
  for (int16_t i = 0; i < 3; i++) {
    bytesOut[4 * i]     = DXL_LOBYTE(DXL_LOWORD(int32Values[i]));
    bytesOut[4 * i + 1] = DXL_HIBYTE(DXL_LOWORD(int32Values[i]));
    bytesOut[4 * i + 2] = DXL_LOBYTE(DXL_HIWORD(int32Values[i]));
    bytesOut[4 * i + 3] = DXL_HIBYTE(DXL_HIWORD(int32Values[i]));
  }
  return bytesOut;
}

/* Byte Array to Float Function  **********************************************/
float bytesToFloat(byte byte1, byte byte2, byte byte3, byte byte4) {
  int32_t inInt = ((byte4 & 0xFF) << 24 | (byte3 & 0xFF) << 16 | (byte2 & 0xFF) << 8 | (byte1 & 0xFF));
  float outFloat = (inInt / 10000.0);
  return outFloat;
}
