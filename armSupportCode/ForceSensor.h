/* This establishes the force sensor class.

   Created 10/27/2020
   by erick nunez
*/

#ifndef FORCE_SENSOR_H
#define FORCE_SENSOR_H

class ForceSensor {
  public:
          ForceSensor(const float xyzSens[3], const float weight, const float forceLimit, const float T);
    void  SensorConfig();
    void  CalibrateSensor();
    float GetGlobalForces(float q1, float q4);
    
  private:
    void  ReadForceSensor();
    void  CheckForces();
    void  FilterForces();
    
    float       _xyzCALIBRATION[3] = 0.0f;
    int16_t     _SENSORSTATUS;
    const float _xyzSENSITIVITY{[3]
    const float _WEIGHT;
    const float _FORCELIMIT;
    const float _DELTAT;
    float xyzRaw_M[3]     = 0.0f;
    float xyzLastRaw_M[3] = 0.0f;
    float xyzFilt_M[3]    = 0.0f;
    float xyzLastFilt_M[3]= 0.0f;
    float xyzGlobal_M[3]  = 0.0f;
};

#endif // FORCE_SENSOR_H
