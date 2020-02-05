/* These functions are used to solve the forward or inverse kinematics of the system in X and Y directions.
   forwardKine() takes the motor counts and outputs the position(SI) for the system.
   inverseKine() takes the position(SI) and outputs the motor counts for the new position.
   
   Created 1/24/2019
   Script by erick nunez
*/

modelSpace forwardKine(jointSpace Q){
  // motor counts/speed --> forwardKine() --> position/velocity(SI)
  modelSpace M;
  // Compute the XY positions from angles 
  M.x = L1_LINK * cos(Q.q1) + L2_LINK * cos(Q.q1 + Q.q4);
  M.y = L1_LINK * sin(Q.q1) + L2_LINK * sin(Q.q1 + Q.q4);

  // M.x = (A1_LINK + A2_LINK) * cos(Q.q1) + L1_LINK * cos(Q.q1) * cos(Q.q2) + L2_LINK * cos(Q.q1 + Q.q4);
  // M.y = (A1_LINK + A2_LINK) * sin(Q.q1) + L1_LINK * sin(Q.q1) * cos(Q.q2) + L2_LINK * sin(Q.q1 + Q.q4);
  // M.z = L1_LINK * sin(Q.q2);
  
  // Multiply velocities with Jacobian Matrix to find the XY velocities
  // xDot = Q1Dot*J11 + Q4Dot*J12
  // yDot = Q1Dot*J21 + Q4Dot*J22
  M.xDot = Q.q1Dot * (-L1_LINK * sin(Q.q1) - L2_LINK * sin(Q.q1 + Q.q4)) + Q.q4Dot * (-L2_LINK * sin(Q.q1 + Q.q4));
  M.yDot = Q.q1Dot * ( L1_LINK * cos(Q.q1) + L2_LINK * cos(Q.q1 + Q.q4)) + Q.q4Dot * ( L2_LINK * cos(Q.q1 + Q.q4));

  // float J11 = - (A1_LINK + A2_LINK) * sin(Q.q1) - L1_LINK * sin(Q.q1) * cos(Q.q2) - L2_LINK * sin(Q.q1 + Q.q4);
  // float J12 = - L1_LINK * cos(Q.q1) * sin(Q.q2);
  // float J13 = - L2_LINK * sin(Q.q1 + Q.q4);
  // float J21 = (A1_LINK + A2_LINK) * cos(Q.q1) + L1_LINK * cos(Q.q1) * cos(Q.q2) + L2_LINK * cos(Q.q1 + Q.q4);
  // float J22 = - L1_LINK * sin(Q.q1) * sin(Q.q2);
  // float J23 = L2_LINK * cos(Q.q1 + Q.q4);
  // float J32 = L1_LINK * cos(Q.q2);
  // J31 = J33 = 0 
  // M.xDot = Q.q1Dot * J11 + Q.q2Dot * J12 + Q.q4Dot * J13;
  // M.yDot = Q.q1Dot * J21 + Q.q2Dot * J22 + Q.q4Dot * J23;
  // M.zDot = Q.q2Dot * J32;
  
  return M;
}

jointSpace inverseKine(modelSpace M){
  // Limits
  static int32_t Q1_MIN = SHOULDER_MIN_POS * DEGREES_PER_COUNT* (PI / 180);
  static int32_t Q1_MAX = SHOULDER_MAX_POS * DEGREES_PER_COUNT* (PI / 180);
  // position/velocity(SI) --> inverseKine() --> motor counts/speed
  jointSpace Q;
  // Solve for joint angles
  Q.q4 = acos((pow(M.x,2) + pow(M.y,2) - pow(L1_LINK,2) - pow(L2_LINK,2))/(2.0 * L1_LINK * L2_LINK));
  Q.q1 = atan2(M.y, M.x) - asin((L2_LINK * sin(Q.q4))/sqrt(pow(M.x,2) + pow(M.y,2)));
  if (Q.q1 < 0) {
    Q.q1 += 2*PI;
  }

  // Solve for joint angular velocities (psuedo inverse Jacobian)
  // invJ = 1/(J11*J22 - J12*J21)*[J22 -J12; -J21 J11]
  // Q1Dot = Xdot*invJ11 + Ydot*invJ12
  // Q4Dot = Xdot*invJ21 + Ydot*invJ22
  //Q.q4Dot = (M.xDot * (L2_LINK * cos(Q.q1 + Q.q4)) + M.yDot * (L2_LINK * sin(Q.q1 + Q.q4)))/(L1_LINK * L2_LINK * sin(Q.q4));
  //Q.q1Dot = (M.xDot * (L1_LINK * cos(Q.q1) - L2_LINK * cos(Q.q1 + Q.q4)) + M.yDot * (-L1_LINK * sin(Q.q1) - L2_LINK * sin(Q.q1 + Q.q4)))/(L1_LINK * L2_LINK * sin(Q.q4));
  
  float detJ = (-L1_LINK * sin(Q.q1) - L2_LINK * sin(Q.q1 + Q.q4))*(L2_LINK * cos(Q.q1 + Q.q4)) - (-L2_LINK * sin(Q.q1 + Q.q4))*(L1_LINK * cos(Q.q1) + L2_LINK * cos(Q.q1 + Q.q4));
  
  Q.q1Dot = (M.xDot*(L2_LINK * cos(Q.q1 + Q.q4)) + M.yDot*(L2_LINK * sin(Q.q1 + Q.q4)))/detJ;
  Q.q4Dot = (M.xDot*(-L1_LINK * cos(Q.q1) - L2_LINK * cos(Q.q1 + Q.q4)) + M.yDot*(-L1_LINK * sin(Q.q1) - L2_LINK * sin(Q.q1 + Q.q4)))/detJ;

  return Q;
}
