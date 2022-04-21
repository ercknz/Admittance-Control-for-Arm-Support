/* Utility functions used with the arm support robot.
   
   For serial communication, data is multipled by 1000.0 to get intergers.
   Then data is divided by 1000.0 to get actual value.

   Created 10/28/2020
   by Erick Nunez
*/

#include <DynamixelSDK.h>
#include "UtilityFunctions.h"

/* ---------------------------------------------------------------------------------------/
/ Bytes to Counts Converter --------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
int16_t bytesToCounts(byte hByte, byte lByte) {
  int16_t decimal = hByte * 256 + lByte;
  return decimal;
}

/* ---------------------------------------------------------------------------------------/
/ Byte Array to Float Function -----------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
float bytesToFloat(byte byte1, byte byte2, byte byte3, byte byte4) {
  int32_t inInt = ((byte4 & 0xFF) << 24 | (byte3 & 0xFF) << 16 | (byte2 & 0xFF) << 8 | (byte1 & 0xFF));
  float outFloat = (inInt / 1000.0);
  return outFloat;
}

/* ---------------------------------------------------------------------------------------/
/ Float Array to Byte Array Function -----------------------------------------------------/
/----------------------------------------------------------------------------------------*/
byte * floatArrayToBytes(float * floatValues) {
  int32_t int32Val;
  static byte bytesOut[12];
  for (int16_t i = 0; i < 3; i++) {
    int32Val = (int32_t)(floatValues[i] * 1000.0);
    bytesOut[4 * i]     = DXL_LOBYTE(DXL_LOWORD(int32Val));
    bytesOut[4 * i + 1] = DXL_HIBYTE(DXL_LOWORD(int32Val));
    bytesOut[4 * i + 2] = DXL_LOBYTE(DXL_HIWORD(int32Val));
    bytesOut[4 * i + 3] = DXL_HIBYTE(DXL_HIWORD(int32Val));
  }
  return bytesOut;
}

/* ---------------------------------------------------------------------------------------/
/ Single Float to Byte Array Function ----------------------------------------------------/
/----------------------------------------------------------------------------------------*/
byte * floatToBytes(float floatValue){
  int32_t int32Data = (int32_t)(floatValue * 1000.0);
  static byte bytesOut[4];
  bytesOut[0] = DXL_LOBYTE(DXL_LOWORD(int32Data));
  bytesOut[1] = DXL_HIBYTE(DXL_LOWORD(int32Data));
  bytesOut[2] = DXL_LOBYTE(DXL_HIWORD(int32Data));
  bytesOut[3] = DXL_HIBYTE(DXL_HIWORD(int32Data));
  return bytesOut;
}

/* ---------------------------------------------------------------------------------------/
/ Int32 Array to Byte Array Function -----------------------------------------------------/
/----------------------------------------------------------------------------------------*/
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
