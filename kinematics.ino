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
  M.x = SHOULDER_ELBOW_LINK * cos(Q.q1) + ELBOW_SENSOR_LINK * cos(Q.q1 + Q.q2);
  M.y = SHOULDER_ELBOW_LINK * sin(Q.q1) + ELBOW_SENSOR_LINK * sin(Q.q1 + Q.q2);
  
  // Multiply velocities with Jacobian Matrix to find the XY velocities
  // xDot = Q1Dot*J11 + Q2Dot*J12
  // yDot = Q1Dot*J21 + Q2Dot*J22
  M.xDot = Q.q1Dot * (-SHOULDER_ELBOW_LINK * sin(Q.q1) - ELBOW_SENSOR_LINK * sin(Q.q1 + Q.q2)) + Q.q2Dot * (-ELBOW_SENSOR_LINK * sin(Q.q1 + Q.q2));
  M.yDot = Q.q1Dot * ( SHOULDER_ELBOW_LINK * cos(Q.q1) + ELBOW_SENSOR_LINK * cos(Q.q1 + Q.q2)) + Q.q2Dot * ( ELBOW_SENSOR_LINK * cos(Q.q1 + Q.q2));
  
  return M;
}

jointSpace inverseKine(modelSpace M){
  // position/velocity(SI) --> inverseKine() --> motor counts/speed
  jointSpace Q;
  // Solve for joint angles
  Q.q2 = acos((pow(M.x,2) + pow(M.y,2) - pow(SHOULDER_ELBOW_LINK,2) - pow(ELBOW_SENSOR_LINK,2))/(2.0 * SHOULDER_ELBOW_LINK * ELBOW_SENSOR_LINK));
  Q.q1 = atan2(M.y, M.x) - asin((ELBOW_SENSOR_LINK * sin(Q.q2))/sqrt(pow(M.x,2) + pow(M.y,2)));
  if (Q.q1 < 0) {
    Q.q1 += 2*PI;
  }

  // Solve for joint angular velocities (psuedo inverse Jacobian)
  // invJ = 1/(J11*J22 - J12*J21)*[J22 -J12; -J21 J11]
  // Q1Dot = Xdot*invJ11 + Ydot*invJ12
  // Q2Dot = Xdot*invJ21 + Ydot*invJ22
  //Q.q2Dot = (M.xDot * (ELBOW_SENSOR_LINK * cos(Q.q1 + Q.q2)) + M.yDot * (ELBOW_SENSOR_LINK * sin(Q.q1 + Q.q2)))/(SHOULDER_ELBOW_LINK * ELBOW_SENSOR_LINK * sin(Q.q2));
  //Q.q1Dot = (M.xDot * (SHOULDER_ELBOW_LINK * cos(Q.q1) - ELBOW_SENSOR_LINK * cos(Q.q1 + Q.q2)) + M.yDot * (-SHOULDER_ELBOW_LINK * sin(Q.q1) - ELBOW_SENSOR_LINK * sin(Q.q1 + Q.q2)))/(SHOULDER_ELBOW_LINK * ELBOW_SENSOR_LINK * sin(Q.q2));
  
  float detJ = (-SHOULDER_ELBOW_LINK * sin(Q.q1) - ELBOW_SENSOR_LINK * sin(Q.q1 + Q.q2))*(ELBOW_SENSOR_LINK * cos(Q.q1 + Q.q2)) - (-ELBOW_SENSOR_LINK * sin(Q.q1 + Q.q2))*(SHOULDER_ELBOW_LINK * cos(Q.q1) + ELBOW_SENSOR_LINK * cos(Q.q1 + Q.q2));
  Q.q1Dot = (M.xDot*(ELBOW_SENSOR_LINK * cos(Q.q1 + Q.q2))                                    + M.yDot*(ELBOW_SENSOR_LINK * sin(Q.q1 + Q.q2)))/detJ;
  Q.q2Dot = (M.xDot*(-SHOULDER_ELBOW_LINK * cos(Q.q1) - ELBOW_SENSOR_LINK * cos(Q.q1 + Q.q2)) + M.yDot*(-SHOULDER_ELBOW_LINK * sin(Q.q1) - ELBOW_SENSOR_LINK * sin(Q.q1 + Q.q2)))/detJ;

  return Q;
}
