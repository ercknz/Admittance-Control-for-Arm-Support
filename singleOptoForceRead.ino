/* This function is used to read a single Optoforce 3 axis sensor.
   The sensor would send a packet with force counts in X, Y and Z.
   The data packet length is 16 bytes and this function reads 32 bytes to account 
   for any shifting. 
   
   Created 12/3/2018
   Script by erick nunez
*/

void singleOptoForceRead(int16_t &xRaw, int16_t &yRaw, int16_t &zRaw, float &FxRaw, float &FyRaw, float &FzRaw){
  byte rawPacket[32];
  byte goodPacket[16];
  static byte header[4] = {170, 7, 8, 10};
  // Reads 2xpacket length for incase of a packet shift
  while (Serial1.available() < 32){}
  for (i = 0; i<32; i++){
    rawPacket[i] = Serial1.read();
  }
  // Searches for good packet
  for (i=0; i<32-12; i++) {
    if (rawPacket[i] == header[0]){
      if (rawPacket[i+1] == header[1]){
        if (rawPacket[i+2] == header[2]){
          if (rawPacket[i+3] == header[3]){
            for (j=0; j<16; j++){
              goodPacket[j]=rawPacket[i+j];
            }
            xRaw = bytesToCounts(goodPacket[8], goodPacket[9]);
            yRaw = bytesToCounts(goodPacket[10], goodPacket[11]);
            zRaw = bytesToCounts(goodPacket[12], goodPacket[13]);
            
            FxRaw = xRaw/xSensitivity - xCal;
            FyRaw = yRaw/ySensitivity - yCal;
            FzRaw = zRaw/zSensitivity - zCal;
          }
        }
      }
    }
  }
}
