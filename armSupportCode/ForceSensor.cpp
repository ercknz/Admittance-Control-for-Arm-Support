/* This establishes the force sensor class.
   This class assumes the sensor is in the Serial 1 port.
   This class allows the configuration the OptoForce Sensor and reads from it.
   Other member functions include a force check for spikes, exponential filter, and global
   force calculation based on robot end effector position.
   The values used by default are 1000Hz for the frequency (option 1 for speed), 1.5Hz
   cut-off frequency (option 6 for filter), and 255 for zeroing the sensor.
   Refer to the Optoforce User Manual Force Sensor DAQ for more information.

   Class arrays use the following:
   Force[3] = {Fx, Fy, Fz};

   Exponential filter used:
   y(n) = w*x(n) + (1-w)*y(n-1)
   where: y(n) is the output value
          w is the filter weight between
          x(n) is the input value
          y(n-1) is the last output

   Created 10/27/2020
   by erick nunez
*/

#include "ForceSensor.h"

/******************** Force Sensor Constructor  ***********************************************************************/
ForceSensor::ForceSensor(HardwareSerial *ptrSer, const float xyzSens[3], const float mass, const float weight, const float accLimit, const float dT) {
  for(int i=0; i<3; i++){
    _xyzSENSITIVITY[i]   = xyzSens[i];
  }
  _WEIGHT     = weight;
  _FORCELIMIT = (mass * accLimit) / dT;
  _DELTAT     = dT;
  SensorPort_M = ptrSer;
}

/******************** Force Sensor Configuration  ***********************************************************************/
void ForceSensor::SensorConfig(const int BaudRate) {
  _BAUDRATE = baudrate;
  SensorPort_M->begin(_BAUDRATE);
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
    SensorPort_M->write(configPacket[i]);
  }
}

/******************** Force Sensor Calibration  ***********************************************************************/
void ForceSensor::CalibrateSensor() {
  for(int i=0; i<3; i++){
    _xyzCALIBRATION[i] = 0.0f;
  }
  static float samples = 2000.0;
  for (int i = 0; i < samples; i++) {
    ReadForceSensor();
    for(int i=0; i<3; i++){
      _xyzCALIBRATION[i] += xyzRaw_M[i] / samples;
    }
  }
}

/******************** Force Sensor Reading  ***********************************************************************/
void ForceSensor::ReadForceSensor() {
  for(int i=0; i<3; i++){
    xyzLastRaw_M[i] = xyzRaw_M[i];
  }
  byte rawPacket[32];
  byte goodPacket[16];
  static byte header[4] = {170, 7, 8, 10};
  while (SensorPort_M->available()) {
    SensorPort_M->read();
  }
  while (SensorPort_M->available() < 32) {} // Reads 2xpacket length incase of a packet shift
  for (int i = 0; i < 32; i++) {
    rawPacket[i] = SensorPort_M->read();
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
            _SENSORSTATUS = bytesToCounts(goodPacket[6], goodPacket[7]); 

            int16_t xCts = bytesToCounts(goodPacket[8], goodPacket[9]);
            int16_t yCts = bytesToCounts(goodPacket[10], goodPacket[11]);
            int16_t zCts = bytesToCounts(goodPacket[12], goodPacket[13]);

            xyzRaw_M[0] = (xCts / _xyzSENSITIVITY[0]) - _xyzCALIBRATION[0];
            xyzRaw_M[1] = (yCts / _xyzSENSITIVITY[1]) - _xyzCALIBRATION[1];
            xyzRaw_M[2] = (zCts / _xyzSENSITIVITY[2]) - _xyzCALIBRATION[2];
          }
        }
      }
    }
  }
  CheckForces();
  FilterForces();
}

/******************** Force Sensor Readings Check  ***********************************************************************/
void ForceSensor::CheckForces() {
  for(int i=0; i<3; i++){
    if (abs((xyzRaw_M[i] - xyzLastRaw_M[i]) / _DELTAT) > _FORCELIMIT) xyzRaw_M[i] = xyzLastRaw_M[i];
  }
}

/******************** Force Sensor Filter  ***********************************************************************/
void ForceSensor::FilterForces() {
  for(int i=0; i<3; i++){
    xyzLastFilt_M[i] = xyzFilt_M[i];
  }
  for(int i=0; i<3; i++){
    xyzFilt_M[i] = _WEIGHT * xyzRaw_M[i] + (1 - _WEIGHT) * xyzLastFilt_M[i];
  }
}

/******************** Force Sensor Global Forces  ***********************************************************************/
float ForceSensor::GetGlobalForces(float q1, float q4) {
  ReadForceSensor();
  xyzGlobal_M[0] = xyzFilt_M[0] * ( sin(q1 + q4)) + xyzFilt_M[1] * (-cos(q1 + q4));
  xyzGlobal_M[0] = xyzFilt_M[0] * (-cos(q1 + q4)) + xyzFilt_M[1] * (-sin(q1 + q4));
  xyzGlobal_M[0] = -xyzFilt_M[2];
  return xyzGlobalM[3];
}
