/* Utility functions used with the arm support robot.

   Created 10/28/2020
   by Erick Nunez
*/

#ifndef UTILITY_FUNCTIONS_H
#define UTILITY_FUNCTIONS_H

int16_t bytesToCounts(byte hByte, byte lByte);

byte * floatArrayToBytes(float * floatValues);

byte * floatToBytes(float floatValue);

byte * int32ArrayToBytes(int32_t * int32Values);

float bytesToFloat(byte byte1, byte byte2, byte byte3, byte byte4);

#endif // UTILITY_FUNCTIONS_H
