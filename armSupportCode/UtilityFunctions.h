/* Utility functions used with the arm support robot.

   Created 10/28/2020
   by Erick Nunez
*/

#ifndef UTILITY_FUNCTIONS_H
#define UTILITY_FUNCTIONS_H

int16_t bytesToCounts(byte hByte, byte lByte);

int32_t * floatToIntArray(float * floatData);

byte * int32ToByteArray(int32_t * int32Data);

float bytesToFloat(byte byte1, byte byte2, byte byte3, byte byte4);

#endif // UTILITY_FUNCTIONS_H
