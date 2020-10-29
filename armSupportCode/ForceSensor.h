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
    void  ReadForceSensor();
    void  CheckForces();
    void  FilterForces();
    void  GetGlobal(float q1, float q4);
  private:
    float _xCal, _yCal, _zCal = 0.0f;
    int16_t _sensorStatus;
    const float _xSensitivity;
    const float _ySensitivity;
    const float _zSensitivity;
    const float _filterWeight;
    const float _forceLimit;
    const float _deltaT;
    float xRaw,     yRaw,     zRaw      = 0.0f;
    float xLastRaw, yLastRaw, zLastRaw  = 0.0f;
    float xFilt,    yFilt,    zFilt     = 0.0f;
    float xLastFilt,yLastFilt,zLastFilt = 0.0f;
    float xGlobal,  yGlobal,  zGlobal   = 0.0f;
};

#endif // FORCE_SENSOR_H
