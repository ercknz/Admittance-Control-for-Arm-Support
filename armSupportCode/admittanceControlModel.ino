/* This function is the admittance control function. it takes a XYZ force input and output XYZ position and velocity.
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

   Created 1/24/2019
   Script by erick nunez
*/

modelSpace admittanceControlModel (forceStruct F, modelSpace init) {
  modelSpace goal;
  // Coefficents and Solution for X-Direction /////////////////////////////////////////////////////
  float Cx1 = ((F.X / DAMPING) - init.xDot) * (MASS / DAMPING);
  float Cx2 = init.x - Cx1;
  goal.x    = Cx1 * exp(-(DAMPING / MASS) * MODEL_DT) + (F.X / DAMPING) * MODEL_DT + Cx2;
  goal.xDot = (F.X / DAMPING) - (DAMPING / MASS) * Cx1 * exp(-(DAMPING / MASS) * MODEL_DT);

  // Coefficents and Solution for Y-Direction //////////////////////////////////////////////////////
  float Cy1 = ((F.Y / DAMPING) - init.yDot) * (MASS / DAMPING);
  float Cy2 = init.y - Cy1;
  goal.y    = Cy1 * exp(-(DAMPING / MASS) * MODEL_DT) + (F.Y / DAMPING) * MODEL_DT + Cy2;
  goal.yDot = (F.Y / DAMPING) - (DAMPING / MASS) * Cy1 * exp(-(DAMPING / MASS) * MODEL_DT);

  // Coefficents and Solution for Z-Direction //////////////////////////////////////////////////////
  float Cz1 = ((1/DAMPING)*(F.Z -GRAVITY * MASS) - init.zDot) * (MASS / DAMPING);
  float Cz2 = init.z - Cz1;
  goal.z    = Cz1 * exp(-(DAMPING / MASS) * MODEL_DT) + (1/DAMPING)*(F.Z - GRAVITY * MASS) * MODEL_DT + Cz2;
  goal.zDot = -(DAMPING / MASS) * Cz1 * exp(-(DAMPING / MASS) * MODEL_DT) + (1/DAMPING)*(F.Z - GRAVITY * MASS);

  return goal;
}
