/* Wrapper class for Absolute Orientation BNO-055 IMU sensor.

   Created 3/21/2022
   by erick nunez
*/

#include <Arduino.h>
#include "IMUWrapper.h"
#include "UtilityFunctions.h"

/* ---------------------------------------------------------------------------------------/
/ IMU Wrapper Contructor  ----------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
IMUWrapper::IMUWrapper(HardwareSerial *ptrSer, const int baudrate) 
  : _BAUDRATE{baudrate}
{
  m_SensorPort = ptrSer;
}

float* IMUWrapper::GetOrientation() {
  return m_orientation;
} 

byte* IMUWrapper::GetOrientationBytes() {
  return m_orientationBytes;
}
 
void IMUWrapper::UpdateOrientation() {
  memcpy(m_lastOrientationBytes, m_orientationBytes, sizeof(m_orientationBytes));
  byte rawPacket[36];
  byte goodPacket[18];
  byte temp[4];
  uint16_t SumCheck;
  uint16_t CHECKSUM;
  while (m_SensorPort->available()) {
    m_SensorPort->read();
  }
  while (m_SensorPort->available() < 36) {} // Reads 2xpacket length incase of a packet shift
  for (int i = 0; i < 36; i++) {
    rawPacket[i] = m_SensorPort->read();
  }
  // Searches for good packet
  for (int i = 0; i < 36 - 14; i++) {
    for (int j = 0; j < 4; j++){
      temp[j] = rawPacket[i + j];
    }
    if (memcmp(_READHEADER, temp, sizeof(_READHEADER)) == 0) {
      for (int j = 0; j < 18; j++) {
        goodPacket[j] = rawPacket[i + j];
      }
      CHECKSUM = bytesToCounts(goodPacket[16], goodPacket[17]);
      SumCheck = 0;
      for (int j = 0; j < 16; j++) {
        SumCheck += goodPacket[j];
      }
      if (SumCheck == CHECKSUM) {

        for (int j = 0; j < 12; i++){
          m_orientationBytes[i] = goodPacket[j + 4];
        }

      } else {
        memcpy(m_orientationBytes, m_lastOrientationBytes, sizeof(m_orientationBytes));
        while (m_SensorPort->available()) {
          m_SensorPort->read();
        }
      }
      break;
    }
  }
}


