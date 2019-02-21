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
                        Cy2 = yPresentPosition - B1
   solutions:           x(t) = Cy1*exp(-(B/M)*t) + (Fy/B)*t + Cy2
                        x'(t) = -(B/M)*Cy1*exp(-(B/M)*t) + (Fy/B)
   Z Direction ***********************************************************************
   2nd order eqn:       M*z" = Fz - B*z' - Mg
   general solution:    xg(t) = Cz1*exp(-(B/M)*t) + Cz2
   particular solution: xp(t) = (Fz/B)*t 
   coefficents:         Cz1 = ((Fz/B) - zPresentVelocity)*(M/B)
                        Cz2 = zPresentPosition - Cz1
   solutions:           x(t) = Cz1*exp(-(B/M)*t) + (Fz/B)*t + Cz2
                        x'(t) = -(B/M)*Cz1*exp(-(B/M)*t) + (Fz/B)
   
   Created 1/24/2019
   Script by erick nunez
*/

void admittanceControl (double xForce, double yForce, 
                        float xPresVelSI,  float yPresVelSI, 
                        float &xGoalPosSI, float &xGoalVelSI, float &yGoalPosSI, float &yGoalVelSI){
  // Coefficents and Solution for X-Direction /////////////////////////////////////////////////////
  float Cx1 = ((xForce/DAMPING) - xPresVelSI)*(MASS/DAMPING);
  float Cx2 = - Cx1;
  xGoalPosSI = Cx1*exp(-(DAMPING/MASS)*TIME) + (xForce/DAMPING)*TIME + Cx2;
  xGoalVelSI = -(DAMPING/MASS)*Cx1*exp(-(DAMPING/MASS)*TIME) + (xForce/DAMPING);
  
  // Coefficents and Solution for Y-Direction //////////////////////////////////////////////////////
  float Cy1 = ((yForce/DAMPING) - yPresVelSI)*(MASS/DAMPING);
  float Cy2 = - Cy1;
  yGoalPosSI = Cy1*exp(-(DAMPING/MASS)*TIME) + (yForce/DAMPING)*TIME + Cy2;
  yGoalVelSI = -(DAMPING/MASS)*Cy1*exp(-(DAMPING/MASS)*TIME) + (yForce/DAMPING);
  
  /* / Coefficents and Solution for Z-Direction //////////////////////////////////////////////////////
  float Cz1 = ((zForce/DAMPING) - zPresVelSI)*(MASS/DAMPING);
  float Cz2 = zPresPosSI - Cz1;
  zGoalPosSI = Cz1*exp(-(DAMPING/MASS)*TIME) + (zForce/DAMPING)*TIME + Cz2;
  zGoalVelSI = -(DAMPING/MASS)*Cz1*exp(-(DAMPING/MASS)*TIME) + (zForce/DAMPING); */
}
