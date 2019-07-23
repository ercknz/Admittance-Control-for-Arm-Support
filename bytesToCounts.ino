/* This function is used to convert the bytes read from the optoforce sensor to force counts.
   
   Created 12/4/2018
   Script by erick nunez
*/

int16_t bytesToCounts(byte hByte, byte lByte){
  static int16_t decimal;
  decimal = hByte * 256 + lByte;
  return decimal;
}
