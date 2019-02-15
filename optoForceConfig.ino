/* This function is used to configure the OptoForce Sensor
   the values used by default are 1000Hz for the frequency (option 1 for speed), 1.5Hz
   cut-off frequency (option 6 for filter), and 255 for zeroing the sensor. Refer to the Optoforce
   User Manual Force Sensor DAQ. This function will not zero the sensor and a seperate function is used
   for calibration. See calibrateForceSensor. 
   
   Created 12/3/2018
   Script by erick nunez
*/

int configPacket[9];
int configPacketSize = 9;
uint8 returnPacket[32];
int returnPacketSize = 32;

void optoForceConfig(){
  SerialUSB.println(".....configuring.....");
  //header
  configPacket[0] = 170;   configPacket[1] = 0;   configPacket[2] = 50;   configPacket[3] = 3;
  // speed
  configPacket[4] = 1;
  // filter, change this if need and check sum accordingly. 
  configPacket[5] = 6;
  // zero
  configPacket[6] = 0;
  // check sum 
  configPacket[7] = 0;     configPacket[8] = 230;
  // write config packet
  for (i = 0; i<configPacketSize; i++){
    Serial1.write(configPacket[i]);
  }
  // read return packet
  /*
  while (Serial1.available() < returnPacketSize){}
  for (i = 0; i<returnPacketSize; i++){
    returnPacket[i] = Serial1.read();
  }
  for (i = 0; i<returnPacketSize; i++){
    SerialUSB.println(returnPacket[i]);
  }
  */
}


