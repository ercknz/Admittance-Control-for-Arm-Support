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

AdmittanceModel::AdmittanceModel(float M, float B, float G, float T) {
  _mass       = M;
  _damping    = B;
  _gravity    = G;
  _deltaT     = T;
}

void AdmittanceModel::InitializeModel(float X, float Y, float Z) {
  xNew = X;
  yNew = Y;
  zNew = Z;
  xDotNew = 0.0;
  yDotNew = 0.0;
  zDotNew = 0.0;
}

void AdmittanceModel::UpdateModel(float Fx, float Fy, float Fz) {
  xInit = xNew;
  yInit = yNew;
  zInit = zNew;
  xDotInit = xDotNew;
  tDotInit = yDotNew;
  zDotInit = zDotNew;
  
  // Coefficents and Solution for X-Direction /////////////////////////////////////////////////////
  float Cx1 = ((Fx / DAMPING) - xDotInit) * (_mass / _damping);
  float Cx2 = xInit - Cx1;
  xNew      = Cx1 * exp(-(_damping / _mass) * _deltaT) + (Fx / _damping) * _deltaT + Cx2;
  xDotNew   = (Fx / _damping) - (_damping / _mass) * Cx1 * exp(-(_damping / _mass) * _deltaT);

  // Coefficents and Solution for Y-Direction //////////////////////////////////////////////////////
  float Cy1 = ((Fy / _damping) - yDotInit) * (_mass / _damping);
  float Cy2 = yInit - Cy1;
  yNew      = Cy1 * exp(-(_damping / _mass) * _deltaT) + (Fy / _damping) * _deltaT + Cy2;
  yDotNew   = (Fy / _damping) - (_damping / _mass) * Cy1 * exp(-(_damping / _mass) * _deltaT);

  // Coefficents and Solution for Z-Direction //////////////////////////////////////////////////////
  float Cz1 = ((1 / _damping) * (Fz - _gravity * _mass) - zDotInit) * (_mass / _damping);
  float Cz2 = zInit - Cz1;
  zNew      = Cz1 * exp(-(_damping / _mass) * _deltaT) + (1 / _damping) * (Fz - _gravity * _mass) * _deltaT + Cz2;
  zDotNew   = -(_damping / _mass) * Cz1 * exp(-(_damping / _mass) * _deltaT) + (1 / _damping) * (Fz - _gravity * _mass);
}

