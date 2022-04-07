/* Wrapper class for Absolute Orientation BNO-055 IMU sensor.

   Created 3/21/2022
   by erick nunez
*/

#ifndef IMU_WRAPPER_H
#define IMU_WRAPPER_H

#include <Arduino.h>

class IMUWrapper {
  public:
           IMUWrapper(HardwareSerial *ptrSer, const int baudrate);
    float* GetOrientation();
    byte*  GetOrientationBytes();
    void   UpdateOrientation();
    
  protected:
    HardwareSerial *m_SensorPort;
    const int   _BAUDRATE;
    const byte  _READHEADER[4] = {220, 10, 150, 50};
    const int   _RX_PKT_LEN = 20;
    const int   _DOUBLE_RX_LEN = 40;
    const int   _RX_DATA_START = 5;

    float m_orientation[3] = { 0.0f };
    byte  m_orientationBytes[12];
    byte  m_lastOrientationBytes[12] = { 0 };
    
};

#endif //IMU_WRAPPER_H
