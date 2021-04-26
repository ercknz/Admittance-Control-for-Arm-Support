/* This class is the admittance control model.
   It takes a XYZ force input and output XYZ position and velocity based on initial conditions.

   Class arrays use the following:
   xyz[3]     = {x, y, x};
   xyzDot[3]  = {xDot, yDot, zDot};

   X Direction *****************************************************************
   2nd order eqn:       M*x" = Fx - B*x'
   general solution:    xg(t) = Cx1*exp(-(B/M)*t) + Cx2
   particular solution: xp(t) = (Fx/B)*t
   coefficents:         Cx1 = ((Fx/B) - xPresentVelocity)*(M/B)
                        Cx2 = xPresentPosition - Cx1
   solutions:           x(t) = Cx1*exp(-(B/M)*t) + (Fx/B)*t + Cx2
                        x'(t) = -(B/M)*Cx1*exp(-(B/M)*t) + (Fx/B)
   Y Direction *****************************************************************
   2nd order eqn:       M*y" = Fy - B*y'
   general solution:    xg(t) = Cy1*exp(-(B/M)*t) + Cy2
   particular solution: xp(t) = (Fy/B)*t
   coefficents:         Cy1 = ((Fy/B) - yPresentVelocity)*(M/B)
                        Cy2 = yPresentPosition - Cy1
   solutions:           x(t) = Cy1*exp(-(B/M)*t) + (Fy/B)*t + Cy2
                        x'(t) = -(B/M)*Cy1*exp(-(B/M)*t) + (Fy/B)
   Z Direction *****************************************************************
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

#include <Arduino.h>
#include "AdmittanceModel.h"

/* Admittance Model Constructor  **********************************************/
AdmittanceModel::AdmittanceModel(float Mxy, float Mz, float Bxy, float Bz, const float G, const float T)
  : _MASS{Mxy, Mz},
    _DAMPING{Bxy, Bz},
    _GRAVITY{G},
    _DELTAT{T}
{
}

/* Admittance Model Initalizer  ***********************************************/
void AdmittanceModel::SetPosition(float *newXYZ) {
  for(int i=0; i<3; i++){
    xyzGoal_M[i]     = newXYZ[i];
  }
}

/* Admittance Model Updater  **************************************************/
void AdmittanceModel::UpdateModel(float *forceXYZ, float springFz, float *externalFxyz) {
  for (int i = 0; i < 3; i++) {
    xyzInit_M[i]    = 0.0f;
    xyzDotInit_M[i] = xyzDotGoal_M[i];
  }

  /* Coefficents and Solution for X-Direction */
  float Fx = forceXYZ[0] + externalFxyz[0];
  float Cx1 = ((Fx / _DAMPING[0]) - xyzDotInit_M[0]) * (_MASS[0] / _DAMPING[0]);
  float Cx2 = xyzInit_M[0] - Cx1;
  xyzGoal_M[0]    = Cx1 * exp(-(_DAMPING[0] / _MASS[0]) * _DELTAT) + (Fx / _DAMPING[0]) * _DELTAT + Cx2;
  xyzDotGoal_M[0] = (Fx / _DAMPING[0]) - (_DAMPING[0] / _MASS[0]) * Cx1 * exp(-(_DAMPING[0] / _MASS[0]) * _DELTAT);

  /* Coefficents and Solution for Y-Direction */
  float Fy = forceXYZ[1] + externalFxyz[1];
  float Cy1 = ((Fy / _DAMPING[0]) - xyzDotInit_M[1]) * (_MASS[0] / _DAMPING[0]);
  float Cy2 = xyzInit_M[1] - Cy1;
  xyzGoal_M[1]    = Cy1 * exp(-(_DAMPING[0] / _MASS[0]) * _DELTAT) + (Fy / _DAMPING[0]) * _DELTAT + Cy2;
  xyzDotGoal_M[1] = (Fy / _DAMPING[0]) - (_DAMPING[0] / _MASS[0]) * Cy1 * exp(-(_DAMPING[0] / _MASS[0]) * _DELTAT);

  /* Coefficents and Solution for Z-Direction */
  float Fz = forceXYZ[2] + springFz + externalFxyz[2];
  float Cz1 = ((Fz  / _DAMPING[1]) - xyzDotInit_M[2]) * (_MASS[1] / _DAMPING[1]);
  float Cz2 = xyzInit_M[2] - Cz1;
  xyzGoal_M[2]    = Cz1 * exp(-(_DAMPING[1] / _MASS[1]) * _DELTAT) + (Fz / _DAMPING[1]) * _DELTAT + Cz2;
  xyzDotGoal_M[2] = (Fz / _DAMPING[1]) - (_DAMPING[1] / _MASS[1]) * Cz1 * exp(-(_DAMPING[1] / _MASS[1]) * _DELTAT);
}

/* Admittance Model Get Functions   *******************************************/
float* AdmittanceModel::GetGoalPos() {
  return xyzGoal_M;
}

float* AdmittanceModel::GetGoalVel() {
  return xyzDotGoal_M;
}

float*  AdmittanceModel::GetMass(){
  return _MASS;
}

float*  AdmittanceModel::GetDamping(){
  return _DAMPING;
}

/* Admittance Model Setter Functions ******************************************/
void AdmittanceModel::SetMassXY(float newMxy){
  if (newMxy > 0.1){
    _MASS[0] = newMxy;
  } else {
    _MASS[0] = 0.1;
  }
}

void AdmittanceModel::SetMassZ(float newMz){
  if (newMz > 0.1){
    _MASS[1] = newMz;
  } else {
    _MASS[1] = 0.1;
  }
}

void AdmittanceModel::SetDampingXY(float newBxy){
  _DAMPING[0] = abs(newBxy);
}

void AdmittanceModel::SetDampingZ(float newBz){
  _DAMPING[1] = abs(newBz);
}
