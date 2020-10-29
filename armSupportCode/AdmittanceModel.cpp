/* This class is the admittance control model.
   It takes a XYZ force input and output XYZ position and velocity based on initial conditions.

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

#include "AdmittanceModel.h"

AdmittanceModel::AdmittanceModel(const float M, const float B, const float G, const float T) {
  _MASS       = M;
  _DAMPING    = B;
  _GRAVITY    = G;
  _DELTAT     = T;
}

void AdmittanceModel::InitializeModel(float XYZ[3]) {
  xyzNew_M[0] = XYZ[0];
  xyzNew_M[1] = XYZ[1];
  xyzNew_M[2] = XYZ[2];
  xyzDotNew_M[0] = 0.0f;
  xyzDotNew_M[1] = 0.0f;
  xyzDotNew_M[2] = 0.0f; 
}

void AdmittanceModel::UpdateModel(float forceXYZ[3]) {
  xyzInit_M[0]    = xyzNew_M[0];
  xyzInit_M[1]    = xyzNew_M[1];
  xyzInit_M[2]    = xyzNew_M[2];
  xyzDotInit_M[0] = xyzDotNew_M[0];
  xyzDotInit_M[1] = xyzDotNew_M[1];
  xyzDotInit_M[2] = xyzDotNew_M[2];
  
  // Coefficents and Solution for X-Direction /////////////////////////////////////////////////////
  float Cx1 = ((Fx / _DAMPING) - xyzDotInit_M[0]) * (_MASS / _DAMPING);
  float Cx2 = xyzInit_M[0] - Cx1;
  xyzNew_M[0]      = Cx1 * exp(-(_DAMPING / _MASS) * _DELTAT) + (Fx / _DAMPING) * _DELTAT + Cx2;
  xyzDotNew_M[0]   = (Fx / _DAMPING) - (_DAMPING / _MASS) * Cx1 * exp(-(_DAMPING / _MASS) * _DELTAT);

  // Coefficents and Solution for Y-Direction //////////////////////////////////////////////////////
  float Cy1 = ((Fy / _DAMPING) - xyzDotInit_M[1]) * (_MASS / _DAMPING);
  float Cy2 = xyzInit_M[1] - Cy1;
  xyzNew_M[1]      = Cy1 * exp(-(_DAMPING / _MASS) * _DELTAT) + (Fy / _DAMPING) * _DELTAT + Cy2;
  xyzDotNew_M[1]   = (Fy / _DAMPING) - (_DAMPING / _MASS) * Cy1 * exp(-(_DAMPING / _MASS) * _DELTAT);

  // Coefficents and Solution for Z-Direction //////////////////////////////////////////////////////
  float Cz1 = ((1 / _DAMPING) * (Fz - _GRAVITY * _MASS) - xyzDotInit_M[2]) * (_MASS / _DAMPING);
  float Cz2 = xyzInit_M[2] - Cz1;
  xyzNew_M[2]      = Cz1 * exp(-(_DAMPING / _MASS) * _DELTAT) + (1 / _DAMPING) * (Fz - _GRAVITY * _MASS) * _DELTAT + Cz2;
  xyzDotNew_M[2]   = -(_DAMPING / _MASS) * Cz1 * exp(-(_DAMPING / _MASS) * _DELTAT) + (1 / _DAMPING) * (Fz - _GRAVITY * _MASS);
}

float Admittance::GetNewPos(){
  return xyzNew_M[3];
}

float Admittance::GetNewVel(){
  return xyzDotNew_M[3];
}

