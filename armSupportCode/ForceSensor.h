/* This establishes the force sensor class.

   Created 10/27/2020
   by erick nunez
*/

#ifndef FORCE_SENSOR_H
#define FORCE_SENSOR_H

class ForceSensor {
  public:
    float xRaw,     yRaw,     zRaw      = 0.0;
    float xLastRaw, yLastRaw, zLastRaw  = 0.0
    float xFilt,    yFilt,    zFilt     = 0.0;
    float xLastFilt,yLastFilt,zLastFilt = 0.0;
    float xGlobal,  yGlobal,  zGlobal   = 0.0;
    ForceSensor(const float xSens, const float ySens, const float zSens, const float weight, const float threshold, const float T);
    void SensorConfig();
    void CalibrateSensor();
    void ReadForceSensor();
    void CheckForces();
    void FilterForces();
    void GetGlobal(float q1, float q4);
  private:
    float _xCal, _yCal, _zCal = 0.0;
    int16_t _sensorStatus;
    const float _xSensitivity;
    const float _ySensitivity;
    const float _zSensitivity;
    const float _filterWeight;
    const float _forceThreshold;
    const float _deltaT;
}

#endif // FORCE_SENSOR_H
