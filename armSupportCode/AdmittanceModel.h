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
    //void  InitializeModel(float XYZ);
    void  UpdateModel(float *forceXYZ);
    float* GetGoalPos();
    float* GetGoalVel();
    
  private: 
    const float _MASS;
    const float _DAMPING;
    const float _GRAVITY;
    const float _DELTAT;
    float xyzGoal_M[3]    = {0.0f};
    float xyzDotGoal_M[3] = {0.0f};
    float xyzInit_M[3]    = {0.0f};
    float xyzDotInit_M[3] = {0.0f};
};

#endif // ADMITTANCE_MODEL_H
