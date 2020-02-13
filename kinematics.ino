/* These functions are used to solve the forward or inverse kinematics of the system in X and Y directions.
   forwardKine() takes the motor counts and outputs the position(SI) for the system.
   inverseKine() takes the position(SI) and outputs the motor counts for the new position.

   Created 1/24/2019
   Script by erick nunez
*/

modelSpace forwardKine(jointSpace Q) {
  // motor counts/speed --> forwardKine() --> position/velocity(SI)
  modelSpace M;
  // Compute the XY positions from angles
  M.x = (A1_LINK + A2_LINK) * cos(Q.q1) + L1_LINK * cos(Q.q1) * cos(Q.q2) + LINK_OFFSET * sin(Q.q1 + Q.q4) + L2_LINK * cos(Q.q1 + Q.q4);
  M.y = (A1_LINK + A2_LINK) * sin(Q.q1) + L1_LINK * sin(Q.q1) * cos(Q.q2) - LINK_OFFSET * cos(Q.q1 + Q.q4) + L2_LINK * sin(Q.q1 + Q.q4);
  M.z = L1_LINK * sin(Q.q2);

  // Multiply velocities with Jacobian Matrix to find the XY velocities
  float J11 = - (A1_LINK + A2_LINK) * sin(Q.q1) - L1_LINK * sin(Q.q1) * cos(Q.q2) + LINK_OFFSET * cos(Q.q1 + Q.q4) - L2_LINK * sin(Q.q1 + Q.q4);
  float J12 = - L1_LINK * cos(Q.q1) * sin(Q.q2);
  float J13 = LINK_OFFSET * cos(Q.q1 + Q.q4) - L2_LINK * sin(Q.q1 + Q.q4);
  float J21 = (A1_LINK + A2_LINK) * cos(Q.q1) + L1_LINK * cos(Q.q1) * cos(Q.q2) + LINK_OFFSET * sin(Q.q1 + Q.q4) + L2_LINK * cos(Q.q1 + Q.q4);
  float J22 = - L1_LINK * sin(Q.q1) * sin(Q.q2);
  float J23 = LINK_OFFSET * sin(Q.q1 + Q.q4) + L2_LINK * cos(Q.q1 + Q.q4);
  float J32 = L1_LINK * cos(Q.q2);  /* and J31 = J33 = 0 */
  M.xDot = Q.q1Dot * J11 + Q.q2Dot * J12 + Q.q4Dot * J13;
  M.yDot = Q.q1Dot * J21 + Q.q2Dot * J22 + Q.q4Dot * J23;
  M.zDot = Q.q2Dot * J32;

  return M;
}

jointSpace inverseKine(jointSpace pres, modelSpace &M) {
  /* position/velocity(SI) --> inverseKine() --> motor counts/speed */
  jointSpace Q;
  
  /* Limits */
  static float Q1_MIN = SHOULDER_MIN_POS * DEGREES_PER_COUNT * (PI / 180); 
  static float Q1_MAX = SHOULDER_MAX_POS * DEGREES_PER_COUNT * (PI / 180); 
  static float Q4_MIN = ELBOW_MIN_POS * DEGREES_PER_COUNT * (PI / 180); 
  static float Q4_MAX = ELBOW_MAX_POS * DEGREES_PER_COUNT * (PI / 180); 
  static float Z_LIMIT = 0.400; 
  static float INNER_DIA = A1_LINK + L1_LINK + A2_LINK - L2_LINK; 
  
  /* Check Z limits */
  if (M.z > Z_LIMIT) {
    M.z = Z_LIMIT;
  } else if (M.z < -Z_LIMIT) {
    M.z = -Z_LIMIT;
  }
  
  /* Find variables based on Z */
  Q.q2 = asin(M.z/L1_LINK);
  float L1_XY = sqrt(pow(L1_LINK,2) - pow(M.z,2)); 
  
  /* Checks if X and Y are both 0 */
  if ((abs(M.x) == 0.0) && (abs(M.y) == 0.0)) {
    Q.q1 = pres.q1;
    Q.q4 = PI;
    M = forwardKine(Q);
    return Q;
  } 
  
  /* Checks walls */
  float R = sqrt(pow(M.x,2) + pow(M.y,2));
  float alpha = atan2(M.y, M.x);
  if (alpha < 0) {
    alpha += 2*PI;
  }
  if (R < INNER_DIA){
    M.x = INNER_DIA * cos(alpha);
    M.y = INNER_DIA * sin(alpha);
    R = INNER_DIA;
  }
  float OUTER_DIA = A1_LINK + L1_XY + A2_LINK + H_OF_L2;
  if (R > OUTER_DIA) {
    M.x = OUTER_DIA * cos(alpha);
    M.y = OUTER_DIA * sin(alpha);
    R = OUTER_DIA;
  }
  
  /* Finds and checks Elbow Angle */
  float gamma = acos((pow((A1_LINK + L1_XY + A2_LINK),2) + pow(H_OF_L2,2) - pow(M.x,2) - pow(M.y,2))/(2 * H_OF_L2 * (A1_LINK + L1_XY + A2_LINK)));
  Q.q4 = PI - gamma + PHI;
  if (Q.q4 + Q4_MIN < Q4_MIN) {
    Q.q4 = PHI;
  } else if (Q.q4 + Q4_MIN > Q4_MAX) {
    Q.q4 = PI;
  }
  
  /* Finds and checks shoulder angle */
  float beta = asin((H_OF_L2 * sin(gamma))/(sqrt(pow(M.x,2) + pow(M.y,2))));
  Q.q1 = alpha - beta;
  if (Q.q1 < Q1_MIN) {
    Q.q1 = Q1_MIN;
  } else if (Q.q1 > Q1_MAX) {
    Q.q1 = Q1_MAX;
  }
  
  /* Checks XYZ */
  modelSpace checkM = forwardKine(Q);
  if ((abs(M.x - checkM.x) > 0.001) || (abs(M.y - checkM.y) > 0.001) || (abs(M.y - checkM.y) > 0.001)){
    M = checkM;
  }
  
  return Q;

  /* Solve for joint angular velocities (psuedo inverse Jacobian) */
  float detJ = (-L1_LINK * sin(Q.q1) - L2_LINK * sin(Q.q1 + Q.q4)) * (L2_LINK * cos(Q.q1 + Q.q4)) - (-L2_LINK * sin(Q.q1 + Q.q4)) * (L1_LINK * cos(Q.q1) + L2_LINK * cos(Q.q1 + Q.q4));
  Q.q1Dot = (M.xDot * (L2_LINK * cos(Q.q1 + Q.q4)) + M.yDot * (L2_LINK * sin(Q.q1 + Q.q4))) / detJ;
  Q.q2Dot = 0;
  Q.q4Dot = (M.xDot * (-L1_LINK * cos(Q.q1) - L2_LINK * cos(Q.q1 + Q.q4)) + M.yDot * (-L1_LINK * sin(Q.q1) - L2_LINK * sin(Q.q1 + Q.q4))) / detJ;

  return Q;
}
