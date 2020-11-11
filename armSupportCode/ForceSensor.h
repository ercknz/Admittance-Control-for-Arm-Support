/* This establishes the force sensor class.

   Created 10/27/2020
   by erick nunez
*/

#ifndef FORCE_SENSOR_H
#define FORCE_SENSOR_H

#include <Arduino.h>

class ForceSensor {
  public:
          ForceSensor(HardwareSerial *ptrSer, const int baudrate, const float xyzSens[3], const float mass, const float weight, const float accLimit, const float dT);
    void  SensorConfig();
    void  CalibrateSensor();
    void  CalculateGlobalForces(float q1, float q4);
    float* GetRawF();
    float* GetGlobalF();
    
  private:
    void  ReadForceSensor();
    void  CheckForces();
    void  FilterForces();

    const int   _BAUDRATE;  
    HardwareSerial *SensorPort_M;
    float       _xyzCALIBRATION[3] = {0.0f};
    int16_t     _SENSORSTATUS;
    const float _xyzSENSITIVITY[3];
    const float _WEIGHT;
    const float _FORCELIMIT;
    const float _DELTAT;
    float xyzRaw_M[3]     = {0.0f};
    float xyzLastRaw_M[3] = {0.0f};
    float xyzFilt_M[3]    = {0.0f};
    float xyzLastFilt_M[3]= {0.0f};
    float xyzGlobal_M[3]  = {0.0f};

//    ForceStruct Raw_M       = {0.0f};
//    ForceStruct LastRaw_M   = {0.0f};
//    ForceStruct Filt_M      = {0.0f};
//    ForceStruct LastFilt_M  = {0.0f};
//    ForceStruct Global_M    = {0.0f};
};

#endif // FORCE_SENSOR_H
