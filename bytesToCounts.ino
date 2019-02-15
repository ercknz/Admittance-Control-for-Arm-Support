/* This function is used to convert the bytes read from the optoforce sensor to force counts.
   
   Created 12/4/2018
   Script by erick nunez
*/

int16 bytesToCounts(uint8 hByte, uint8 lByte){
  int16 decimal = hByte * 256 + lByte;
  return decimal;
}
