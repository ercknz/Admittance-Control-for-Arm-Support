/* This establishes the force sensor class.

   Created 10/27/2020
   by erick nunez
*/

#ifndef FORCE_SENSOR_H
#define FORCE_SENSOR_H

#include <Arduino.h>

class ForceSensor {
  public:
          ForceSensor(HardwareSerial *ptrSer, const int baudrate, const float filterWeight);
    void  SensorConfig();
    void  CalibrateSensor();
    void  CalculateGlobalForces(float *q);
    float* GetRawF();
    float* GetGlobalF();
    float GetForceFilter();
    void   SetFilter(float newFilterValue);
    
  protected:
    void  ReadForceSensor();
    void  FilterForces();

    HardwareSerial *SensorPort_M;
    const int   _BAUDRATE;  
    float       _xyzCALIBRATION[3] = {0.0f};
    int16_t     _SAMPLECOUNTER;
    int16_t     _SENSORSTATUS;
    const byte  _READHEADER[4] = {170, 7, 8, 10};
    const float _xyzSENSITIVITY[3];
    const float _posXsens[3];
    const float _posYsens[3];
    const float _posZsens[3];
    const float _negXsens[3];
    const float _negYsens[3];
    const float _negZsens[3];
    float FilterWeight_M;
    float xyzRaw_M[3]     = {0.0f};
    float xyzLastRaw_M[3] = {0.0f};
    float xyzFilt_M[3]    = {0.0f};
    float xyzLastFilt_M[3]= {0.0f};
    float xyzGlobal_M[3]  = {0.0f};
    bool calibratingFlag_M = false;
};

#endif // FORCE_SENSOR_H
