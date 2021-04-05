/* This class is the admittance control model.
   It takes a XYZ force input and output XYZ position and velocity based on initial conditions.

   Created 10/27/2020
   by erick nunez
*/

#ifndef ADMITTANCE_MODEL_H
#define ADMITTANCE_MODEL_H

class AdmittanceModel {
  public:
           AdmittanceModel(float Mxy, float Mz, float Bxy, float Bz, const float G, const float T);
    void   SetPosition(float *newXYZ);
    void   UpdateModel(float *forceXYZ);
    float* GetGoalPos();
    float* GetGoalVel();
    void   SetMassXY(float newMxy);
    void   SetMassZ(float newMz);
    void   SetDampingXY(float newBxy);
    void   SetDampingZ(float newBz);

  private:
    const float _GRAVITY, _DELTAT;
    float _MASS_XY, _MASS_Z, _DAMPING_XY, _DAMPING_Z;

    float xyzGoal_M[3]    = {0.0f};
    float xyzDotGoal_M[3] = {0.0f};
    float xyzInit_M[3]    = {0.0f};
    float xyzDotInit_M[3] = {0.0f};
};

#endif // ADMITTANCE_MODEL_H
