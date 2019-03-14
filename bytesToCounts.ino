/* This function is used to convert the bytes read from the optoforce sensor to force counts.
   
   Created 12/4/2018
   Script by erick nunez
*/

int16_t bytesToCounts(uint8_t hByte, uint8_t lByte){
  int16_t decimal = hByte * 256 + lByte;
  return decimal;
}
