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

#include <Arduino.h>
#include "UtilityFunctions.h"
#include "ForceSensor.h"
#include "armSupportNamespace.h"

/* ---------------------------------------------------------------------------------------/
/ Force Sensor Constructor ---------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
ForceSensor::ForceSensor(HardwareSerial *ptrSer, const int baudrate, const float filterWeight)
  : _BAUDRATE{baudrate},
    _xyzSENSITIVITY{ASR::xyzSens[0], ASR::xyzSens[1], ASR::xyzSens[2]},
    _posXsens{ASR::posX_SM[0], ASR::posX_SM[1], ASR::posX_SM[2]},
    _posYsens{ASR::posY_SM[0], ASR::posY_SM[1], ASR::posY_SM[2]},
    _posZsens{ASR::posZ_SM[0], ASR::posZ_SM[1], ASR::posZ_SM[2]},
    _negXsens{ASR::negX_SM[0], ASR::negX_SM[1], ASR::negX_SM[2]},
    _negYsens{ASR::negY_SM[0], ASR::negY_SM[1], ASR::negY_SM[2]},
    _negZsens{ASR::negZ_SM[0], ASR::negZ_SM[1], ASR::negZ_SM[2]}
{
  SensorPort_M = ptrSer;
  FilterWeight_M = filterWeight;
}

/* ---------------------------------------------------------------------------------------/
/ Force Sensor Memeber Getter and Setter Functions ---------------------------------------/
/----------------------------------------------------------------------------------------*/
float* ForceSensor::GetRawF() {
  return xyzRaw_M;
}

float* ForceSensor::GetGlobalF() {
  return xyzGlobal_M;
} 

float ForceSensor::GetForceFilter(){
  return FilterWeight_M;
}

void ForceSensor::SetFilter(float newFilterValue){
  if (newFilterValue > 0.99) {
    FilterWeight_M = 0.99;
    return;
  }
  if (newFilterValue < 0.1) {
    FilterWeight_M = 0.1;
    return;
  }
  FilterWeight_M = newFilterValue;
}

/* ---------------------------------------------------------------------------------------/
/ Force Sensor Configuration ---------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void ForceSensor::SensorConfig() {
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
  delay(500);
  for (int i = 0; i < 9; i++) {
    SensorPort_M->write(configPacket[i]);
  }
  delay(500);
}

/* ---------------------------------------------------------------------------------------/
/ Force Sensor Calibration ---------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void ForceSensor::CalibrateSensor() {
  calibratingFlag_M = true;
  float newXYZcal[3] = {0.0};
  for (int i = 0; i < 3; i++) {
    _xyzCALIBRATION[i] = 0.0f;
  }
  float samples = 2000.0;
  for (int i = 0; i < samples; i++) {
    ReadForceSensor();
    for (int j = 0; j < 3; j++) {
      newXYZcal[j] += xyzRaw_M[j] / samples;
    }
  }
  for (int i = 0; i < 3; i++) {
    _xyzCALIBRATION[i] = newXYZcal[i];
  }
  calibratingFlag_M = false;
}

/* ---------------------------------------------------------------------------------------/
/ Force Sensor Reading -------------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void ForceSensor::ReadForceSensor() {
  for (int i = 0; i < 3; i++) {
    xyzLastRaw_M[i] = xyzRaw_M[i];
  }
  byte rawPacket[32];
  byte goodPacket[16];
  byte temp[4];
  int16_t xCts, yCts, zCts;
  uint16_t SumCheck;
  uint16_t CHECKSUM;
  while (SensorPort_M->available()) {
    SensorPort_M->read();
  }
  while (SensorPort_M->available() < 32); // Reads 2xpacket length incase of a packet shift
  for (int i = 0; i < 32; i++) {
    rawPacket[i] = SensorPort_M->read();
  }
  // Searches for good packet
  for (int i = 0; i < 32 - 12; i++) {
    for (int j = 0; j < 4; j++){
      temp[j] = rawPacket[i+j];
    }
    if (memcmp(_READHEADER, temp, sizeof(_READHEADER)) == 0) {
      for (int j = 0; j < 16; j++) {
        goodPacket[j] = rawPacket[i + j];
      }
      CHECKSUM = bytesToCounts(goodPacket[14], goodPacket[15]);
      SumCheck = 0;
      for (int j = 0; j < 14; j++) {
        SumCheck += goodPacket[j];
      }
      if (SumCheck == CHECKSUM) {
        _SAMPLECOUNTER = bytesToCounts(goodPacket[4], goodPacket[5]);
        _SENSORSTATUS = bytesToCounts(goodPacket[6], goodPacket[7]);

        xCts = bytesToCounts(goodPacket[8], goodPacket[9]);
        yCts = bytesToCounts(goodPacket[10], goodPacket[11]);
        zCts = bytesToCounts(goodPacket[12], goodPacket[13]);

        xyzRaw_M[0] = xCts * _xyzSENSITIVITY[0];
        xyzRaw_M[1] = yCts * _xyzSENSITIVITY[1];
        xyzRaw_M[2] = zCts * _xyzSENSITIVITY[2];

        /*
        if (xCts > 0.0) {
          xyzRaw_M[0] = _posXsens[0] * xCts + _posXsens[1] * yCts + _posXsens[2] * zCts;
        } else if (xCts < 0.0) {
          xyzRaw_M[0] = _negXsens[0] * xCts + _negXsens[1] * yCts + _negXsens[2] * zCts;
        } else {
          xyzRaw_M[0] = 0.0;
        }
        
        if (yCts > 0.0) {
          xyzRaw_M[1] = _posYsens[0] * xCts + _posYsens[1] * yCts + _posYsens[2] * zCts;
        } else if (yCts < 0.0) {
          xyzRaw_M[1] = _negYsens[0] * xCts + _negYsens[1] * yCts + _negYsens[2] * zCts;
        } else {
          xyzRaw_M[1] = 0.0;
        }

        if (zCts > 0.0) {
          xyzRaw_M[2] = _posZsens[0] * xCts + _posZsens[1] * yCts + _posZsens[2] * zCts;
        } else if (zCts < 0.0) {
          xyzRaw_M[2] = _negZsens[0] * xCts + _negZsens[1] * yCts + _negZsens[2] * zCts;
        } else {
          xyzRaw_M[2] = 0.0;
        }
        */

      } else {
        for (int j = 0; i < 3; i++) {
          xyzRaw_M[j] = xyzLastRaw_M[j];
        }
        while (SensorPort_M->available()) {
          SensorPort_M->read();
        }
      }
      break;
    }
  }
  if (!calibratingFlag_M) FilterForces();
}

/* ---------------------------------------------------------------------------------------/
/ Force Sensor Filter --------------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void ForceSensor::FilterForces() {
  for (int i = 0; i < 3; i++) {
    xyzLastFilt_M[i] = xyzFilt_M[i];
    xyzFilt_M[i] = FilterWeight_M * (xyzRaw_M[i] - _xyzCALIBRATION[i]) + (1.0 - FilterWeight_M) * xyzLastFilt_M[i];
  }
}

/* ---------------------------------------------------------------------------------------/
/ Force Sensor Global Forces -------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void ForceSensor::CalculateGlobalForces(float *q) {
  // q[] = [q1, q2, q4]
  ReadForceSensor();
  xyzGlobal_M[0] = xyzFilt_M[0] * ( sin(q[0] + q[2])) + xyzFilt_M[1] * (-cos(q[0] + q[2]));
  xyzGlobal_M[1] = xyzFilt_M[0] * (-cos(q[0] + q[2])) + xyzFilt_M[1] * (-sin(q[0] + q[2]));
  xyzGlobal_M[2] = -xyzFilt_M[2];
}
