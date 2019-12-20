/* This function is used to calibrate the OptoForce Sensor. It will collect forces for 2 seconds and 
   will average out the values and save them in the calibration address for X Y and Z. 
   
   Created 12/6/2018
   Script by erick nunez
*/

void calibrateForceSensor(float &xCal, float &yCal, float &zCal){
  static int samples = 2000.0;
  int16_t xRaw, yRaw, zRaw;
  forceStruct F;
  for (int i = 0; i < samples; i++) {
    F = singleOptoForceRead(xCal, yCal, zCal);
    xCal += F.X/samples;
    yCal += F.Y/samples;
    zCal += F.Z/samples;
  }
}
