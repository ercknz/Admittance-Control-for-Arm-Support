/* This establishes the force sensor class.

   Created 10/27/2020
   by erick nunez
*/

#ifndef FORCE_SENSOR_H
#define FORCE_SENSOR_H

#include <Arduino.h>

class ForceSensor {
  public:
          ForceSensor(HardwareSerial *ptrSer, const int baudrate, const float xyzSens[3], const float filterWeight);
    void  SensorConfig();
    void  CalibrateSensor();
    void  CalculateGlobalForces(float q1, float q4);
    float* GetRawF();
    float* GetGlobalF();
    
  private:
    void  ReadForceSensor();
    void  FilterForces();

    HardwareSerial *SensorPort_M;
    const int   _BAUDRATE;  
    float       _xyzCALIBRATION[3] = {0.0f};
    int16_t     _SAMPLECOUNTER;
    int16_t     _SENSORSTATUS;
    const float _xyzSENSITIVITY[3];
    const float _FILTERWEIGHT;
    float xyzRaw_M[3]     = {0.0f};
    float xyzLastRaw_M[3] = {0.0f};
    float xyzFilt_M[3]    = {0.0f};
    float xyzLastFilt_M[3]= {0.0f};
    float xyzGlobal_M[3]  = {0.0f};
};

#endif // FORCE_SENSOR_H
