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
IMUWrapper::IMUWrapper(UARTClass *ptrSer, const int baudrate) 
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

void IMUWrapper::StartComm(){
  m_SensorPort->begin(_BAUDRATE);
}
 
void IMUWrapper::UpdateOrientation() {
  Serial.println("2.1");
  memcpy(m_lastOrientationBytes, m_orientationBytes, sizeof(m_orientationBytes));
  byte rawPacket[_DOUBLE_RX_LEN];
  byte goodPacket[_RX_PKT_LEN];
  byte tempHeader[4];
  uint16_t SumCheck;
  uint16_t CHECKSUM;
  Serial.println("2.2");
  while (m_SensorPort->available()) {
    m_SensorPort->read();
  }
  Serial.println("2.3");
  while (m_SensorPort->available() < _DOUBLE_RX_LEN);// Reads 2xpacket length incase of a packet shift
  Serial.println("2.4");
  for (int i = 0; i < _DOUBLE_RX_LEN; i++) {
    rawPacket[i] = m_SensorPort->read();
  }
  Serial.println("2.5");
  // Searches for good packet
  for (int i = 0; i < _DOUBLE_RX_LEN - 16; i++) {
    for (int j = 0; j < 4; j++){
      tempHeader[j] = rawPacket[i + j];
    }
    if (memcmp(_READHEADER, tempHeader, sizeof(_READHEADER)) == 0) {
      for (int j = 0; j < _RX_PKT_LEN; j++) {
        goodPacket[j] = rawPacket[i + j];
      }
      CHECKSUM = bytesToCounts(goodPacket[_RX_PKT_LEN - 2], goodPacket[_RX_PKT_LEN - 1]);
      SumCheck = 0;
      for (int j = 0; j < _RX_PKT_LEN - 2; j++) {
        SumCheck += goodPacket[j];
      }
      if (SumCheck == CHECKSUM) {
        for (int j = 0; j < 12; i++){
          m_orientationBytes[i] = goodPacket[j + _RX_DATA_START];
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


