/* This class is the admittance control model.
   It takes a XYZ force input and output XYZ position and velocity based on initial conditions.

   Created 10/27/2020
   by erick nunez
*/

#ifndef ADMITTANCE_MODEL_H
#define ADMITTANCE_MODEL_H

class AdmittanceModel {
  public:
          AdmittanceModel(const float M, const float B, const float G, const float T);
    void  InitializeModel(float XYZ[3]);
    void  UpdateModel(float forceXYZ[3]);
    float GetNewPos();
    float GetNewVel();
    
  private:
    const float _MASS;
    const float _DAMPING;
    const float _GRAVITY;
    const float _DELTAT;
    float xyzNew_M[3];
    float xyzDotNew_M[3];
    float xyzInit_M[3];
    float xyzDotInit_M[3];
};

#endif // ADMITTANCE_MODEL_H
