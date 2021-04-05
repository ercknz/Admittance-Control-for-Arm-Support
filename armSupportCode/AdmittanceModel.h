/* This class is the admittance control model.
   It takes a XYZ force input and output XYZ position and velocity based on initial conditions.

   Created 10/27/2020
   by erick nunez
*/

#ifndef ADMITTANCE_MODEL_H
#define ADMITTANCE_MODEL_H

class AdmittanceModel {
  public:
          AdmittanceModel(const float Mxy, const float Bxy, const float Mz, const float Bz, const float G, const float T);
    void   SetPosition(float *newXYZ);
    void   UpdateModel(float *forceXYZ);
    float* GetGoalPos();
    float* GetGoalVel();
    void   SetXYMass(float newXYMass);
    void   SetZMass(float newZMass);
    void   SetXYDampening(float newXYDampening);
    void   SetZDampening(float newZDampening);

  private:
    const float _GRAVITY;
    const float _DELTAT;
    float _MASS_XY;
    float _DAMPING_XY;
    float _MASS_Z;
    float _DAMPING_Z;

    float xyzGoal_M[3]    = {0.0f};
    float xyzDotGoal_M[3] = {0.0f};
    float xyzInit_M[3]    = {0.0f};
    float xyzDotInit_M[3] = {0.0f};
};

#endif // ADMITTANCE_MODEL_H
