/* This class is the admittance control model.
   It takes a XYZ force input and output XYZ position and velocity based on initial conditions.

   Created 10/27/2020
   by erick nunez
*/

#ifndef ADMITTANCE_MODEL_H
#define ADMITTANCE_MODEL_H

class AdmittanceModel {
  public:
    float xNew,     yNew,     zNew;
    float xDotNew,  yDotNew,  zDotNew;
    float xInit,    yInit,    zInit;
    float xDotInit, yDotInit, zDotInit;
    AdmittanceModel(float M, float B, float G, float T);
    void InitializeModel(float X, float Y, float Z);
    void UpdateModel(float Fx, float Fy, float Fz);
  private:
    float _mass;
    float _damping;
    const float _gravity;
    const float _deltaT;
}

#endif // ADMITTANCE_MODEL_H
