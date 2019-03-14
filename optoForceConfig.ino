/* This function is used to configure the OptoForce Sensor
   the values used by default are 1000Hz for the frequency (option 1 for speed), 1.5Hz
   cut-off frequency (option 6 for filter), and 255 for zeroing the sensor. Refer to the Optoforce
   User Manual Force Sensor DAQ. This function will not zero the sensor and a seperate function is used
   for calibration. See calibrateForceSensor. 
   
   Created 12/3/2018
   Script by erick nunez
*/

int     configPacket[9];
int     configPacketSize = 9;
uint8_t returnPacket[32];
int     returnPacketSize = 32;
int     packetSum = 0;

void optoForceConfig(){
  SerialUSB.println(".....configuring.....");
  //header
  configPacket[0] = 170;   configPacket[1] = 0;   configPacket[2] = 50;   configPacket[3] = 3;
  // speed
  // 0:noData, 1:1000Hz, 3:333Hz, 10:100Hz, 33:30Hz, 100:10Hz
  configPacket[4] = 1;
  // filter
  // 0:noFilter, 1:500Hz, 2:150Hz, 3:50Hz, 4:15Hz, 5:5Hz, 6:1.5Hz
  configPacket[5] = 6;
  // zero
  // 0:originalValues, 255:zeroSensor
  configPacket[6] = 255;
  // check sum 
  for (i=0; i<7; i++){
    packetSum += configPacket[i];
  }
  configPacket[7] = floor(packetSum/256);     configPacket[8] = floor(packetSum % 256);
  // write config packet
  for (i = 0; i<configPacketSize; i++){
    Serial1.write(configPacket[i]);
  }
}


