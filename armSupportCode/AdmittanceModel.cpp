/* This class is the admittance control model.
   It takes a XYZ force input and output XYZ position and velocity based on initial conditions.

   Class arrays use the following: 
   xyz[3]     = {x, y, x}; 
   xyzDot[3]  = {xDot, yDot, zDot};

   X Direction ********************************************************************
   2nd order eqn:       M*x" = Fx - B*x'
   general solution:    xg(t) = Cx1*exp(-(B/M)*t) + Cx2
   particular solution: xp(t) = (Fx/B)*t
   coefficents:         Cx1 = ((Fx/B) - xPresentVelocity)*(M/B)
                        Cx2 = xPresentPosition - Cx1
   solutions:           x(t) = Cx1*exp(-(B/M)*t) + (Fx/B)*t + Cx2
                        x'(t) = -(B/M)*Cx1*exp(-(B/M)*t) + (Fx/B)
   Y Direction *********************************************************************
   2nd order eqn:       M*y" = Fy - B*y'
   general solution:    xg(t) = Cy1*exp(-(B/M)*t) + Cy2
   particular solution: xp(t) = (Fy/B)*t
   coefficents:         Cy1 = ((Fy/B) - yPresentVelocity)*(M/B)
                        Cy2 = yPresentPosition - Cy1
   solutions:           x(t) = Cy1*exp(-(B/M)*t) + (Fy/B)*t + Cy2
                        x'(t) = -(B/M)*Cy1*exp(-(B/M)*t) + (Fy/B)
   Z Direction ***********************************************************************
   2nd order eqn:       M*z" = Fz - B*z' - Mg
   general solution:    xg(t) = Cz1*exp(-(B/M)*t) + Cz2
   particular solution: xp(t) = (1/B)*(Fz-g*M)*t
   coefficents:         Cz1 = ((1/B)*(Fz-g*M) - zPresentVelocity)*(M/B)
                        Cz2 = zPresentPosition - Cz1
   solutions:           z(t) = Cz1*exp(-(B/M)*t) + (1/B)*(Fz-g*M)*t + Cz2
                        z'(t) = -(B/M)*Cz1*exp(-(B/M)*t) + (1/B)*(Fz-g*M)

   Created 10/27/2020
   by erick nunez
*/

#include <math.h>
#include "AdmittanceModel.h"
//#include "UtilityFunctions.h"

/******************** Admittance Model Constructor  ***********************************************************************/
AdmittanceModel::AdmittanceModel(const float M, const float B, const float G, const float T)
                :_MASS{M}, 
                _DAMPING{B}, 
                _GRAVITY{G}, 
                _DELTAT{T}
{
}

/******************** Admittance Model Initalizer  ***********************************************************************/
//void AdmittanceModel::InitializeModel(float XYZ) {
//  for(int i=0; i<3; i++){
//    xyzGoal_M[i]     = XYZ[i];
//    xyzDotGoal_M[i]  = 0.0f;
//  }
//}

/******************** Admittance Model Updater  ***********************************************************************/
void AdmittanceModel::UpdateModel(float *forceXYZ) {
  for(int i=0; i<3; i++){
    xyzInit_M[i]    = 0.0f;
    xyzDotInit_M[i] = xyzDotGoal_M[i];
  }
  
  // Coefficents and Solution for X-Direction /////////////////////////////////////////////////////
  float Cx1 = ((forceXYZ[0] / _DAMPING) - xyzDotInit_M[0]) * (_MASS / _DAMPING);
  float Cx2 = xyzInit_M[0] - Cx1;
  xyzGoal_M[0]      = Cx1 * exp(-(_DAMPING / _MASS) * _DELTAT) + (forceXYZ[0] / _DAMPING) * _DELTAT + Cx2;
  xyzDotGoal_M[0]   = (forceXYZ[0] / _DAMPING) - (_DAMPING / _MASS) * Cx1 * exp(-(_DAMPING / _MASS) * _DELTAT);

  // Coefficents and Solution for Y-Direction //////////////////////////////////////////////////////
  float Cy1 = ((forceXYZ[1] / _DAMPING) - xyzDotInit_M[1]) * (_MASS / _DAMPING);
  float Cy2 = xyzInit_M[1] - Cy1;
  xyzGoal_M[1]      = Cy1 * exp(-(_DAMPING / _MASS) * _DELTAT) + (forceXYZ[1] / _DAMPING) * _DELTAT + Cy2;
  xyzDotGoal_M[1]   = (forceXYZ[1] / _DAMPING) - (_DAMPING / _MASS) * Cy1 * exp(-(_DAMPING / _MASS) * _DELTAT);

  // Coefficents and Solution for Z-Direction //////////////////////////////////////////////////////
  float Cz1 = ((1 / _DAMPING) * (forceXYZ[2] - _GRAVITY * _MASS) - xyzDotInit_M[2]) * (_MASS / _DAMPING);
  float Cz2 = xyzInit_M[2] - Cz1;
  xyzGoal_M[2]      = Cz1 * exp(-(_DAMPING / _MASS) * _DELTAT) + (1 / _DAMPING) * (forceXYZ[2] - _GRAVITY * _MASS) * _DELTAT + Cz2;
  xyzDotGoal_M[2]   = -(_DAMPING / _MASS) * Cz1 * exp(-(_DAMPING / _MASS) * _DELTAT) + (1 / _DAMPING) * (forceXYZ[2] - _GRAVITY * _MASS);
}

/******************** Admittance Model Get Functions   ***************************************************************/
float* AdmittanceModel::GetGoalPos(){
  return xyzGoal_M;
}

float* AdmittanceModel::GetGoalVel(){
  return xyzDotGoal_M;
}
