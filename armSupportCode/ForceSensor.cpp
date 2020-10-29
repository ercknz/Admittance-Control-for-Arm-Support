/* This establishes the force sensor class.
   This class assumes the sensor is in the Serial 1 port.
   This class allows the configuration the OptoForce Sensor and reads from it.
   Other member functions include a force check for spikes, exponential filter, and global
   force calculation based on robot end effector position.
   The values used by default are 1000Hz for the frequency (option 1 for speed), 1.5Hz
   cut-off frequency (option 6 for filter), and 255 for zeroing the sensor.
   Refer to the Optoforce User Manual Force Sensor DAQ for more information.

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

ForceSensor::ForceSensor(const float xyzSens[3], const float weight, const float forceLimit, const float T) {
  _xSensitivity   = xyzSens[0];
  _ySensitivity   = xyzSens[1];
  _zSensitivity   = xyzSens[2];
  _filterWeight   = weight;
  _forceLimit     = forceLimit;
  _deltaT         = T;
}

void ForceSensor::SensorConfig() {
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

void ForceSensor::CalibrateSensor() {
  _xCal = 0.0f;
  _yCal = 0.0f;
  _zCal = 0.0f;
  static float samples = 2000.0;
  for (int i = 0; i < samples; i++) {
    ReadForceSensor();
    _xCal += xRaw / samples;
    _yCal += yRaw / samples;
    _zCal += zRaw / samples;
  }
}

void ForceSensor::ReadForceSensor() {
  byte rawPacket[32];
  byte goodPacket[16];
  static byte header[4] = {170, 7, 8, 10};
  while (Serial1.available()) {
    Serial1.read();
  }
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
            _sensorStatus = bytesToCounts(goodPacket[6], goodPacket[7]); 

            int16_t xCts = bytesToCounts(goodPacket[8], goodPacket[9]);
            int16_t yCts = bytesToCounts(goodPacket[10], goodPacket[11]);
            int16_t zCts = bytesToCounts(goodPacket[12], goodPacket[13]);

            xRaw = (xCts / _xSensitivity) - _xCal;
            yRaw = (yCts / _ySensitivity) - _yCal;
            zRaw = (zCts / _zSensitivity) - _zCal;
          }
        }
      }
    }
  }
}

void ForceSensor::GetGlobal(float q1, float q4) {
  xGlobal = xRaw * ( sin(q1 + q4)) + yRaw * (-cos(q1 + q4));
  yGlobal = xRaw * (-cos(q1 + q4)) + yRaw * (-sin(q1 + q4));
  zGlobal = -zRaw;
}

void ForceSensor::FilterForces() {
  outputForce.X = weight * inputForce.X + (1 - weight) * currentForce.X;
  outputForce.Y = weight * inputForce.Y + (1 - weight) * currentForce.Y;
  outputForce.Z = weight * inputForce.Z + (1 - weight) * currentForce.Z;
}

void ForceSensor::CheckForces() {
  if (abs((newForces.X - lastForces.X) / _deltaT) > _forceThreshold) checkedForces.X = lastForces.X;
  if (abs((newForces.Y - lastForces.Y) / _deltaT) > _forceThreshold) checkedForces.Y = lastForces.Y;
  if (abs((newForces.Z - lastForces.Z) / _deltaT) > _forceThreshold) checkedForces.Z = lastForces.Z;
}
