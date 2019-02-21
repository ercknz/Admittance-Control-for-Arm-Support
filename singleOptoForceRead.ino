/* This function is used to read a single Optoforce 3 axis sensor.
   The sensor would send a packet with force counts in X, Y and Z.
   The data packet length is 16 bytes and this function reads 32 bytes to account 
   for any shifting. 
   
   Created 12/3/2018
   Script by erick nunez
*/

int rawPacketSize = 32;
uint8 rawPacket[32];
int goodPacketSize = 16;
uint8 goodPacket[16];
uint8 header[4] = {170, 7, 8, 10};

void singleOptoForceRead(){
  while (Serial1.available() < rawPacketSize){}
  for (i = 0; i<rawPacketSize; i++){
    rawPacket[i] = Serial1.read();
  }
  // Searches for good packet
  for (i=0; i<rawPacketSize-12; i++) {
    if (rawPacket[i] == header[0]){
      if (rawPacket[i+1] == header[1]){
        if (rawPacket[i+2] == header[2]){
          if (rawPacket[i+3] == header[3]){
            for (j=0; j<goodPacketSize; j++){
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
