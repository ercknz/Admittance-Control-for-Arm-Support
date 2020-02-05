/* This function is used to read a single Optoforce 3 axis sensor.
   The sensor would send a packet with force counts in X, Y and Z.
   The data packet length is 16 bytes and this function reads 32 bytes to account 
   for any shifting. 
   
   Created 12/3/2018
   Script by erick nunez
*/

forceStruct singleOptoForceRead(float xCal, float yCal, float zCal){
  forceStruct force;
  byte rawPacket[32];
  byte goodPacket[16];
  static byte header[4] = {170, 7, 8, 10};
  int16_t sensorStatus;
  clearSensorBuffer();
  while (Serial1.available() < 32){} // Reads 2xpacket length incase of a packet shift
  for (int i = 0; i<32; i++){
    rawPacket[i] = Serial1.read();
  }
  // Searches for good packet
  for (int i=0; i<32-12; i++) {
    if (rawPacket[i] == header[0]){
      if (rawPacket[i+1] == header[1]){
        if (rawPacket[i+2] == header[2]){
          if (rawPacket[i+3] == header[3]){
            for (int j=0; j<16; j++){
              goodPacket[j]=rawPacket[i+j];
            }
            
            sensorStatus = bytesToCounts(goodPacket[6], goodPacket[7]);
            
            int16_t xRaw = bytesToCounts(goodPacket[8], goodPacket[9]);
            int16_t yRaw = bytesToCounts(goodPacket[10], goodPacket[11]);
            int16_t zRaw = bytesToCounts(goodPacket[12], goodPacket[13]); 

            //Sensor flipped about the y-axis 180 degrees
            force.X = -(xRaw/xSensitivity - xCal);
            force.Y = yRaw/ySensitivity - yCal;
            force.Z = -(zRaw/zSensitivity - zCal);
          }
        }
      }
    }
  }
  return force;
}

void clearSensorBuffer(){
  while (Serial1.available()){
    Serial1.read();
  }
}

