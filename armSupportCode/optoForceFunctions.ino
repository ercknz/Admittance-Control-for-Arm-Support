/* These functions are used to configure the OptoForce Sensor and read from it.
   Other functions are ones needed to accomplish the read and configure.
   The values used by default are 1000Hz for the frequency (option 1 for speed), 1.5Hz
   cut-off frequency (option 6 for filter), and 255 for zeroing the sensor. Refer to the Optoforce
   User Manual Force Sensor DAQ. This function will not zero the sensor and a seperate function is used
   for calibration. See calibrateForceSensor.

   Created 12/3/2018
   Script by erick nunez
*/

/******************** OptoForce Configuration function ************************************************/
void optoForceConfig() {
  byte configPacket[9];
  uint16_t packetSum = 0;
  /* Header */
  configPacket[0] = 170;   configPacket[1] = 0;   configPacket[2] = 50;   configPacket[3] = 3;
  /* Speed -> 0:noData, 1:1000Hz, 3:333Hz, 10:100Hz, 33:30Hz, 100:10Hz */
  configPacket[4] = 1;
  /* Filter -> 0:noFilter, 1:500Hz, 2:150Hz, 3:50Hz, 4:15Hz, 5:5Hz, 6:1.5Hz */
  configPacket[5] = 6;
  /* Zero -> 0:originalValues, 255:zeroSensor */
  configPacket[6] = 255;
  /* Check Sum */
  for (int i = 0; i < 7; i++) {
    packetSum += configPacket[i];
  }
  configPacket[7] = floor(packetSum / 256);
  configPacket[8] = floor(packetSum % 256);

  /* write config packet */
  for (int i = 0; i < 9; i++) {
    Serial1.write(configPacket[i]);
  }
}

/******************** OptoForce Read function ********************************************************/
forceStruct singleOptoForceRead(float xCal, float yCal, float zCal) {
  forceStruct force;
  byte rawPacket[32];
  byte goodPacket[16];
  static byte header[4] = {170, 7, 8, 10};
  int16_t sensorStatus;
  clearSensorBuffer();
  while (Serial1.available() < 32) {} // Reads 2xpacket length incase of a packet shift
  for (int i = 0; i < 32; i++) {
    rawPacket[i] = Serial1.read();
  }
  // Searches for good packet
  for (int i = 0; i < 32 - 12; i++) {
    if (rawPacket[i] == header[0]) {
      if (rawPacket[i + 1] == header[1]) {
        if (rawPacket[i + 2] == header[2]) {
          if (rawPacket[i + 3] == header[3]) {
            for (int j = 0; j < 16; j++) {
              goodPacket[j] = rawPacket[i + j];
            }
            sensorStatus = bytesToCounts(goodPacket[6], goodPacket[7]);

            int16_t xRaw = bytesToCounts(goodPacket[8], goodPacket[9]);
            int16_t yRaw = bytesToCounts(goodPacket[10], goodPacket[11]);
            int16_t zRaw = bytesToCounts(goodPacket[12], goodPacket[13]);

            force.X = xRaw / xSensitivity - xCal;
            force.Y = yRaw / ySensitivity - yCal;
            force.Z = zRaw / zSensitivity - zCal;
          }
        }
      }
    }
  }
  return force;
}

/******************** OptoForce Calibration function ************************************************/
void calibrateForceSensor(float &xCal, float &yCal, float &zCal) {
  static int samples = 2000.0;
  int16_t xRaw, yRaw, zRaw;
  forceStruct F;
  for (int i = 0; i < samples; i++) {
    F = singleOptoForceRead(xCal, yCal, zCal);
    xCal += F.X / samples;
    yCal += F.Y / samples;
    zCal += F.Z / samples;
  }
}

/******************** OptoForce Clear Buffer function ************************************************/
void clearSensorBuffer() {
  while (Serial1.available()) {
    Serial1.read();
  }
}

/******************** OptoForce Check Sum function **************************************************/
int16_t bytesToCounts(byte hByte, byte lByte) {
  int16_t decimal = hByte * 256 + lByte;
  return decimal;
}
