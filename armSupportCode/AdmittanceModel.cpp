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

#include <math.h>
#include "AdmittanceModel.h"

/* Admittance Model Constructor  **********************************************/
AdmittanceModel::AdmittanceModel(float Mxy, float Mz, float Bxy, float Bz, const float G, const float T)
  : _MASS_XY{Mxy},
    _MASS_Z{Mz},
    _DAMPING_XY{Bxy},
    _DAMPING_Z{Bz},
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
void AdmittanceModel::UpdateModel(float *forceXYZ) {
  for (int i = 0; i < 3; i++) {
    xyzInit_M[i]    = 0.0f;
    xyzDotInit_M[i] = xyzDotGoal_M[i];
  }

  /* Coefficents and Solution for X-Direction */
  float Cx1 = ((forceXYZ[0] / _DAMPING_XY) - xyzDotInit_M[0]) * (_MASS_XY / _DAMPING_XY);
  float Cx2 = xyzInit_M[0] - Cx1;
  xyzGoal_M[0]    = Cx1 * exp(-(_DAMPING_XY / _MASS_XY) * _DELTAT) + (forceXYZ[0] / _DAMPING_XY) * _DELTAT + Cx2;
  xyzDotGoal_M[0] = (forceXYZ[0] / _DAMPING_XY) - (_DAMPING_XY / _MASS_XY) * Cx1 * exp(-(_DAMPING_XY / _MASS_XY) * _DELTAT);

  /* Coefficents and Solution for Y-Direction */
  float Cy1 = ((forceXYZ[1] / _DAMPING_XY) - xyzDotInit_M[1]) * (_MASS_XY / _DAMPING_XY);
  float Cy2 = xyzInit_M[1] - Cy1;
  xyzGoal_M[1]    = Cy1 * exp(-(_DAMPING_XY / _MASS_XY) * _DELTAT) + (forceXYZ[1] / _DAMPING_XY) * _DELTAT + Cy2;
  xyzDotGoal_M[1] = (forceXYZ[1] / _DAMPING_XY) - (_DAMPING_XY / _MASS_XY) * Cy1 * exp(-(_DAMPING_XY / _MASS_XY) * _DELTAT);

  /* Coefficents and Solution for Z-Direction */
  float Cz1 = ((forceXYZ[2]  / _DAMPING_Z) - xyzDotInit_M[2]) * (_MASS_Z / _DAMPING_Z);
  float Cz2 = xyzInit_M[2] - Cz1;
  xyzGoal_M[2]    = Cz1 * exp(-(_DAMPING_Z / _MASS_Z) * _DELTAT) + (forceXYZ[2] / _DAMPING_Z) * _DELTAT + Cz2;
  xyzDotGoal_M[2] = (forceXYZ[2] / _DAMPING_Z) - (_DAMPING_Z / _MASS_Z) * Cz1 * exp(-(_DAMPING_Z / _MASS_Z) * _DELTAT);
}

/* Admittance Model Get Functions   *******************************************/
float* AdmittanceModel::GetGoalPos() {
  return xyzGoal_M;
}

float* AdmittanceModel::GetGoalVel() {
  return xyzDotGoal_M;
}

float  AdmittanceModel::GetMassXY(){
  return _MASS_XY;
}

float  AdmittanceModel::GetMassZ(){
  return _MASS_Z;
}

float  AdmittanceModel::GetDampingXY(){
  return _DAMPING_XY;
}

float  AdmittanceModel::GetDampingZ(){
  return _DAMPING_Z;
}

/* Admittance Model Setter Functions ******************************************/
void AdmittanceModel::SetMassXY(float newMxy){
  _MASS_XY += newMxy;
}

void AdmittanceModel::SetMassZ(float newMz){
  _MASS_Z += newMz;
}

void AdmittanceModel::SetDampingXY(float newBxy){
  _DAMPING_XY += newBxy;
}

void AdmittanceModel::SetDampingZ(float newBz){
  _DAMPING_Z += newBz;
}
