/* This function is used to calibrate the OptoForce Sensor. It will collect forces for 2 seconds and 
   will average out the values and save them in the calibration address for X Y and Z. 
   
   Created 12/6/2018
   Script by erick nunez
*/

void calibrateForceSensor(float &xCal, float &yCal, float &zCal){
  float xTotal, yTotal, zTotal;
  int samples = 2000;
  for (k = 0; k < samples; k++) {
    singleOptoForceRead(xRaw, yRaw, zRaw, FxRaw, FyRaw, FzRaw);
    xTotal += Fx;
    yTotal += Fy;
    zTotal += Fz;
  }
  xCal = xTotal/samples;
  yCal = yTotal/samples;
  zCal = zTotal/samples;
  Serial.println(".....Done calibrating sensor.....");
}
